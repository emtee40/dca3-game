#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <filesystem>  // C++17
#include <stdexcept>

#ifdef _MSC_VER
    #define PACKED_STRUCT __pragma(pack(push,1)) struct __pragma(pack(pop))
#else
    #define PACKED_STRUCT struct __attribute__((packed))
#endif

// Structures as specified:

// Matches what the SDT file contains
PACKED_STRUCT tSampleSDT {
    uint32_t nOffset;           // Not necessarily used for writing new data, but read from SDT
    uint32_t nSize;             // Not necessarily used for writing new data, but read from SDT
    uint32_t nFrequency;        // We'll store this into the DSC
    uint32_t nLoopStartInBytes; // We'll convert this to "samples" in DSC
    int32_t  nLoopEnd;          // Not used directly in the DSC, but read from SDT
};

// Matches what we want in the DSC file
PACKED_STRUCT tSample {
    uint32_t nFileOffset;      // in bytes
    uint32_t nByteSize;        // in bytes
    uint32_t nFrequency;       // in hz
    uint32_t nLoopStartSample; // in samples, 0 if no separate loop data
    uint32_t nLoopFileOffset;  // in bytes, 0 if none
    uint32_t nLoopByteSize;    // in bytes, 0 if none
};


static void usage(const char* progName) {
    std::cerr << "Usage: " << progName << " <inputSDT> <outputRAW> <outputDSC> <SFX_DIR>\n";
    std::cerr << "Example:\n";
    std::cerr << "    " << progName << " data.sdt merged.raw output.dsc sfx_folder\n";
}

// Helper function to read the entire contents of a binary file into a vector of bytes
static std::vector<char> readFile(const std::string& filename)
{
    std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
    if (!ifs.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!ifs.read(buffer.data(), size)) {
        throw std::runtime_error("Failed to read file: " + filename);
    }
    return buffer;
}

// Helper function to write a buffer to a binary file (append mode or from start)
static void writeFileAppend(const std::string& filename, const char* data, size_t size, bool append = true)
{
    std::ios_base::openmode mode = std::ios::binary;
    if (append) {
        mode |= std::ios::app;
    } else {
        mode |= std::ios::trunc;
    }
    std::ofstream ofs(filename, mode);
    if (!ofs.is_open()) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    ofs.write(data, size);
}

int main(int argc, char** argv)
{
    if (argc != 5) {
        usage(argv[0]);
        return 1;
    }

    std::string sdtPath   = argv[1];  // input SDT file
    std::string rawPath   = argv[2];  // output RAW file
    std::string dscPath   = argv[3];  // output DSC file
    std::string sfxFolder = argv[4];  // SFX_DIR folder

    // Read the entire SDT file into memory
    std::vector<char> sdtData;
    try {
        sdtData = readFile(sdtPath);
    } catch(const std::exception& e) {
        std::cerr << "Error reading SDT file: " << e.what() << std::endl;
        return 1;
    }

    // Determine how many tSampleSDT entries are in the SDT file
    size_t totalBytes = sdtData.size();
    if (totalBytes % sizeof(tSampleSDT) != 0) {
        std::cerr << "SDT file size is not a multiple of tSampleSDT struct size.\n";
        return 1;
    }
    size_t numSamples = totalBytes / sizeof(tSampleSDT);

    // Parse all tSampleSDT entries
    std::vector<tSampleSDT> sdtEntries(numSamples);
    std::memcpy(sdtEntries.data(), sdtData.data(), totalBytes);

    // We'll build DSC descriptors in memory first
    std::vector<tSample> dscEntries(numSamples);

    // Truncate (or create) the RAW file before we start appending
    {
        std::ofstream ofs(rawPath, std::ios::binary | std::ios::trunc);
        if (!ofs.is_open()) {
            std::cerr << "Error creating/truncating RAW file: " << rawPath << std::endl;
            return 1;
        }
    }

    // Now process each entry in the SDT, find the corresponding sfx_<n>.pcm / sfx_<n>_loop.pcm
    // Concatenate them into the RAW, and fill in the DSC.
    uint64_t currentOffset = 0; // Keep track of where we are in the RAW file

    for (size_t i = 0; i < numSamples; ++i)
    {
        tSampleSDT& sdt  = sdtEntries[i];
        tSample&    desc = dscEntries[i];

        // Prepare file paths
        std::string basePcm  = sfxFolder + "/sfx_" + std::to_string(i) + ".pcm";
        std::string loopPcm  = sfxFolder + "/sfx_" + std::to_string(i) + "_loop.pcm";

        // Read main PCM (sfx_<n>.pcm)
        uint32_t mainOffset    = 0;
        uint32_t mainByteSize  = 0;
        uint32_t loopOffset    = 0;
        uint32_t loopByteSize  = 0;
        uint32_t loopStartSamp = 0;

        // For the main PCM
        try {
            if (std::filesystem::exists(basePcm)) {
                std::vector<char> buffer = readFile(basePcm);
                mainByteSize = static_cast<uint32_t>(buffer.size());
                mainOffset = static_cast<uint32_t>(currentOffset);
                if (buffer.size() & 3) {
                    // Pad to 4-byte boundary
                    size_t padSize = 4 - (buffer.size() & 3);
                    buffer.insert(buffer.end(), padSize, 0);
                    // std::cerr << "Warning: Padded main PCM for index " << i << " with " << padSize << " bytes" << std::endl;
                }
                // Write to RAW
                writeFileAppend(rawPath, buffer.data(), buffer.size(), true);

                // Advance current offset
                currentOffset += buffer.size();
            } else {
                // If the main PCM doesn't exist, you could decide to throw an error or just keep zero
                // For now, let's throw an error
                throw std::runtime_error("Missing PCM file: " + basePcm);
            }
        } catch(const std::exception& e) {
            std::cerr << "Error processing main PCM for index " << i << ": " << e.what() << std::endl;
            return 1;
        }

        // For the loop PCM (sfx_<n>_loop.pcm); it might not exist
        if (std::filesystem::exists(loopPcm)) {
            try {
                std::vector<char> bufferLoop = readFile(loopPcm);
                loopByteSize = static_cast<uint32_t>(bufferLoop.size());
                loopOffset   = static_cast<uint32_t>(currentOffset);

                if (bufferLoop.size() & 3) {
                    // Pad to 4-byte boundary
                    size_t padSize = 4 - (bufferLoop.size() & 3);
                    bufferLoop.insert(bufferLoop.end(), padSize, 0);
                    // std::cerr << "Warning: Padded loop PCM for index " << i << " with " << padSize << " bytes." << std::endl;
                }

                // Write to RAW
                writeFileAppend(rawPath, bufferLoop.data(), bufferLoop.size(), true);

                // Advance current offset
                currentOffset += bufferLoop.size();
            } catch(const std::exception& e) {
                std::cerr << "Error processing loop PCM for index " << i << ": " << e.what() << std::endl;
                return 1;
            }

            // Convert loopStartInBytes from SDT to samples
            // According to the note: "Note each sample in the SDT is indicated by two bytes"
            // So if the SDT says 'nLoopStartInBytes', to get the loop start in samples, divide by 2
            loopStartSamp = sdt.nLoopStartInBytes / 2;
        } else {
            // If there's no loop file, we leave loopOffset, loopByteSize, and loopStartSamp = 0
        }

        // Fill in the tSample descriptor
        desc.nFileOffset      = mainOffset;
        desc.nByteSize        = mainByteSize;
        desc.nFrequency       = sdt.nFrequency;
        desc.nLoopStartSample = loopStartSamp;
        desc.nLoopFileOffset  = loopOffset;
        desc.nLoopByteSize    = loopByteSize;
    }

    // Finally, write the DSC file as a binary array of tSample
    {
        std::ofstream dscOut(dscPath, std::ios::binary | std::ios::trunc);
        if (!dscOut.is_open()) {
            std::cerr << "Error creating DSC file: " << dscPath << std::endl;
            return 1;
        }
        dscOut.write(reinterpret_cast<const char*>(dscEntries.data()),
                     dscEntries.size() * sizeof(tSample));
    }

    std::cout << "Successfully packed " << numSamples << " samples.\n";
    std::cout << "Output RAW: " << rawPath << std::endl;
    std::cout << "Output DSC: " << dscPath << std::endl;
    return 0;
}
