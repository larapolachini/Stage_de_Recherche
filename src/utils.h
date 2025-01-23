#ifndef UTILS_H
#define UTILS_H

void ensure_directories_exist(const std::string& filename);
void delete_files_with_extension(const std::string& path, const std::string& extension, bool recursive = false);
bool string_to_bool( std::string const& str);

#endif // UTILS_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
