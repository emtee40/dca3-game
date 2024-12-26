#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <cassert>

// Define the tSample structure
struct tSample {
    uint32_t nOffset;
    uint32_t nSize;
    uint32_t nFrequency;
    uint32_t nLoopStart;
    int32_t  nLoopEnd;
};

// WAV file header structure
struct WavHeader {
    char riff[4];                // "RIFF"
    uint32_t fileSize;           // File size minus 8 bytes
    char wave[4];                // "WAVE"
    char fmt[4];                 // "fmt "
    uint32_t fmtSize;            // Size of fmt chunk (16 for PCM)
    uint16_t audioFormat;        // Audio format (1 for PCM)
    uint16_t numChannels;        // Number of channels
    uint32_t sampleRate;         // Sample rate
    uint32_t byteRate;           // Byte rate
    uint16_t blockAlign;         // Block align
    uint16_t bitsPerSample;      // Bits per sample
    char data[4];                // "data"
    uint32_t dataSize;           // Size of data chunk
};

void writeWavFile(const std::string &filename, const std::vector<int16_t> &data, uint32_t sampleRate) {
    WavHeader header;

    // Fill in the WAV header fields
    std::memcpy(header.riff, "RIFF", 4);
    header.fileSize = 36 + data.size() * sizeof(int16_t);
    std::memcpy(header.wave, "WAVE", 4);
    std::memcpy(header.fmt, "fmt ", 4);
    header.fmtSize = 16;
    header.audioFormat = 1;  // PCM
    header.numChannels = 1;  // Mono
    header.sampleRate = sampleRate;
    header.byteRate = sampleRate * sizeof(int16_t);
    header.blockAlign = sizeof(int16_t);
    header.bitsPerSample = 16;
    std::memcpy(header.data, "data", 4);
    header.dataSize = data.size() * sizeof(int16_t);

    // Open the output WAV file
    std::ofstream outFile(filename, std::ios::binary);
    if (!outFile) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    // Write the header
    outFile.write(reinterpret_cast<char*>(&header), sizeof(WavHeader));

    // Write the audio data
    outFile.write(reinterpret_cast<const char*>(data.data()), data.size() * sizeof(int16_t));

    outFile.close();
}

int main(int argc, char *argv[]) {
    // Check if enough arguments are provided
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <sample_file> <data_file> <out_folder>" << std::endl;
        return 1;
    }

    // Open the sample file in binary mode
    std::ifstream sampleFile(argv[1], std::ios::binary);
    if (!sampleFile) {
        std::cerr << "Failed to open sample file: " << argv[1] << std::endl;
        return 1;
    }

    // Open the data file in binary mode
    std::ifstream dataFile(argv[2], std::ios::binary);
    if (!dataFile) {
        std::cerr << "Failed to open data file: " << argv[2] << std::endl;
        return 1;
    }

    std::string folder = argv[3];

    // Read and process all tSample structures from the sample file
    tSample sample;
    int recordNumber = 0;

    while (sampleFile.read(reinterpret_cast<char*>(&sample), sizeof(tSample))) {
        // Seek to the correct offset in the data file
        dataFile.seekg(sample.nOffset, std::ios::beg);
        if (!dataFile) {
            std::cerr << "Failed to seek in data file for record " << recordNumber << std::endl;
            continue;
        }

        assert(sample.nSize % sizeof(int16_t) == 0);
        assert(sample.nLoopStart % sizeof(int16_t) == 0);
        assert(sample.nLoopEnd == -1 || sample.nLoopEnd % sizeof(int16_t) == 0);

        // Read the sample data
        std::vector<int16_t> audioData(sample.nSize / sizeof(int16_t));
        if (!dataFile.read(reinterpret_cast<char*>(audioData.data()), sample.nSize)) {
            std::cerr << "Failed to read data for record " << recordNumber << std::endl;
            continue;
        }

        // Generate the WAV file name
        std::string wavFilename = folder + "/sfx_" + std::to_string(recordNumber) + ".wav";
        std::string wavFilenameLoop = folder + "/sfx_" + std::to_string(recordNumber) + "_loop.wav";

        // std::cout << "Sample size: " << sample.nSize << " bytes" << std::endl;
        // std::cout << "Sample frequency: " << sample.nFrequency << " Hz" << std::endl;
        // std::cout << "Sample loop start: " << sample.nLoopStart << std::endl;
        // std::cout << "Sample loop end: " << sample.nLoopEnd << std::endl;
        // std::cout << std::endl;

        // Write the data to a WAV file
        if (sample.nLoopStart != 0) {
            writeWavFile(wavFilename, audioData, sample.nFrequency);
            std::vector<int16_t> loopData(audioData.begin() + sample.nLoopStart/2, audioData.begin() + sample.nLoopEnd/2);
            writeWavFile(wavFilenameLoop, loopData, sample.nFrequency);
        } else {
            writeWavFile(wavFilename, audioData, sample.nFrequency);
        }
        recordNumber++;
    }

    // Close the files
    sampleFile.close();
    dataFile.close();

    return 0;
}