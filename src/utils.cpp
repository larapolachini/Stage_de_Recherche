#include <iostream>
#include <string>
#include <filesystem>
#include <algorithm>


bool string_to_bool( std::string const& str) {
    // Convert string to lowercase for case-insensitive comparison
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);

    if (lowerStr == "true" || lowerStr == "1") {
        return true;
    } else if (lowerStr == "false" || lowerStr == "0") {
        return false;
    } else {
        throw std::invalid_argument("Invalid string for boolean conversion: " + str);
    }
}


void ensure_directories_exist(const std::string& filename) {
    try {
        std::filesystem::path filePath(filename);
        std::filesystem::path directory = filePath.parent_path();

        if (!directory.empty() && !std::filesystem::exists(directory)) {
            std::filesystem::create_directories(directory);
            std::cout << "Created directories: " << directory << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directories for '" << filename << "': " << e.what() << std::endl;
    }
}

void delete_files_with_extension(const std::string& path, const std::string& extension, bool recursive = false) {
    try {
        std::filesystem::path dirPath(path);

        // Check if the path exists and is a directory
        if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath)) {
            std::cerr << "Invalid directory: " << path << std::endl;
            return;
        }

        // Declare an iterator
        if (recursive) {
            for (const auto& entry : std::filesystem::recursive_directory_iterator(dirPath)) {
                if (entry.is_regular_file() && entry.path().extension() == extension) {
                    std::filesystem::remove(entry.path());
                    std::cout << "Deleted: " << entry.path() << std::endl;
                }
            }
        } else {
            for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
                if (entry.is_regular_file() && entry.path().extension() == extension) {
                    std::filesystem::remove(entry.path());
                    std::cout << "Deleted: " << entry.path() << std::endl;
                }
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Filesystem error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
