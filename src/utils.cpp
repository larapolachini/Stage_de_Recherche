#include <iostream>
#include <string>
#include <filesystem>
#include <algorithm>

#include "utils.h"

std::shared_ptr<spdlog::logger> glogger;

void init_logger() {
    // Create a console logger with color support
    glogger = spdlog::stdout_color_mt("console");
    glogger->set_level(spdlog::level::info); // Set default log level
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v"); // Set log format
}


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
            glogger->info("Created directories: {}", directory.string());
        }
    } catch (const std::filesystem::filesystem_error& e) {
        glogger->error("Error creating directories for '{}': {}'", filename, e.what());
    }
}

void delete_files_with_extension(const std::string& path, const std::string& extension, bool recursive) {
    try {
        std::filesystem::path dirPath(path);

        // Check if the path exists and is a directory
        if (!std::filesystem::exists(dirPath) || !std::filesystem::is_directory(dirPath)) {
            glogger->error("Invalid directory: {}", path);
            return;
        }

        // Declare an iterator
        if (recursive) {
            for (const auto& entry : std::filesystem::recursive_directory_iterator(dirPath)) {
                if (entry.is_regular_file() && entry.path().extension() == extension) {
                    std::filesystem::remove(entry.path());
                    glogger->debug("Deleted: {}", entry.path().string());
                }
            }
        } else {
            for (const auto& entry : std::filesystem::directory_iterator(dirPath)) {
                if (entry.is_regular_file() && entry.path().extension() == extension) {
                    std::filesystem::remove(entry.path());
                    glogger->debug("Deleted: {}", entry.path().string());
                }
            }
        }
    } catch (const std::filesystem::filesystem_error& e) {
        glogger->error("Filesystem error: {}", e.what());
    } catch (const std::exception& e) {
        glogger->error("Error: {}", e.what());
    }
}

float euclidean_distance(const b2Vec2& a, const b2Vec2& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
