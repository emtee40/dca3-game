#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <cstring>
#include <iomanip>

// Sector size constant
const size_t SECTOR_SIZE = 2048;

// Record structure for .dir file
struct DirRecord {
    uint32_t offset;
    uint32_t size;
    char name[24];
};

// Function to read .dir file
std::vector<DirRecord> readDirFile(const std::string& dirFilePath) {
    std::ifstream dirFile(dirFilePath, std::ios::binary);
    if (!dirFile) {
        throw std::runtime_error("Could not open .dir file.");
    }

    std::vector<DirRecord> records;
    DirRecord record;

    while (dirFile.read(reinterpret_cast<char*>(&record), sizeof(DirRecord))) {
        records.push_back(record);
    }

    return records;
}

// Function to unpack .img file
void unpackImg(const std::string& baseName, const std::string& dataDir) {
    std::string dirFilePath = baseName + ".dir";
    std::string imgFilePath = baseName + ".img";

    auto records = readDirFile(dirFilePath);

    std::ifstream imgFile(imgFilePath, std::ios::binary);
    if (!imgFile) {
        throw std::runtime_error("Could not open .img file.");
    }

    std::filesystem::create_directories(dataDir);

    for (const auto& record : records) {
        std::string outputFilePath = dataDir + "/" + std::string(record.name);
        std::ofstream outputFile(outputFilePath, std::ios::binary);
        if (!outputFile) {
            throw std::runtime_error("Could not create output file: " + outputFilePath);
        }

        imgFile.seekg(record.offset * SECTOR_SIZE);
        std::vector<char> buffer(record.size * SECTOR_SIZE);
        imgFile.read(buffer.data(), buffer.size());

        outputFile.write(buffer.data(), buffer.size());
    }
}

// Function to create .dir file
void writeDirFile(const std::string& dirFilePath, const std::vector<DirRecord>& records) {
    std::ofstream dirFile(dirFilePath, std::ios::binary);
    if (!dirFile) {
        throw std::runtime_error("Could not create .dir file.");
    }

    for (const auto& record : records) {
        dirFile.write(reinterpret_cast<const char*>(&record), sizeof(DirRecord));
    }
}

// Function to pack .img file
void packImg(const std::string& dataDir, const std::string& baseName) {
    std::string dirFilePath = baseName + ".dir";
    std::string imgFilePath = baseName + ".img";

    std::ofstream imgFile(imgFilePath, std::ios::binary);
    if (!imgFile) {
        throw std::runtime_error("Could not create .img file.");
    }

    std::vector<DirRecord> records;
    uint32_t offset = 0;

    for (const auto& entry : std::filesystem::directory_iterator(dataDir)) {
        if (entry.is_regular_file()) {
            DirRecord record;
            std::memset(&record, 0, sizeof(DirRecord));
            std::strncpy(record.name, entry.path().filename().string().c_str(), sizeof(record.name) - 1);

            std::ifstream inputFile(entry.path(), std::ios::binary | std::ios::ate);
            if (!inputFile) {
                throw std::runtime_error("Could not open input file: " + entry.path().string());
            }

            size_t fileSize = inputFile.tellg();
            inputFile.seekg(0);

            size_t numSectors = (fileSize + SECTOR_SIZE - 1) / SECTOR_SIZE;
            std::vector<char> buffer(numSectors * SECTOR_SIZE, 0);

            inputFile.read(buffer.data(), fileSize);

            record.offset = offset;
            record.size = numSectors;

            imgFile.write(buffer.data(), buffer.size());
            records.push_back(record);

            offset += numSectors;
        }
    }

    writeDirFile(dirFilePath, records);
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: imgtool <unpack|pack> <base name> <data dir>" << std::endl;
        return 1;
    }

    std::string command = argv[1];
    std::string baseName = argv[2];
    std::string dataDir = argv[3];

    try {
        if (command == "unpack") {
            unpackImg(baseName, dataDir);
        } else if (command == "pack") {
            packImg(dataDir, baseName);
        } else {
            std::cerr << "Unknown command: " << command << std::endl;
            return 1;
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}