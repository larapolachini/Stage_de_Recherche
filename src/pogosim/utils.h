#ifndef UTILS_H
#define UTILS_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h> // For colored console output
#include <spdlog/fmt/ostr.h> // Enables << operator for logging

#include <box2d/box2d.h>

#include <random>

// Declare a global logger (shared pointer)
extern std::shared_ptr<spdlog::logger> glogger;
extern std::shared_ptr<spdlog::logger> robotlogger;
void init_logger();
void loggers_add_file_sink(std::string const& filename);

// Random device and generator
extern std::random_device rd;
extern std::mt19937 rnd_gen;

void ensure_directories_exist(const std::string& filename);
void delete_files_with_extension(const std::string& path, const std::string& extension, bool recursive = false);
bool string_to_bool( std::string const& str);

std::string resolve_path(const std::string& inputPath);


#endif // UTILS_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
