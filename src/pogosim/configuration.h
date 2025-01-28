#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "pogosim.h"

class Configuration {
private:
    std::unordered_map<std::string, std::string> config_map;
public:
    void load(const std::string& file_name);
    std::string get(const std::string& key, const std::string& default_value = "") const;
    void set(std::string const& key, std::string const& value);
    bool contains(const std::string& key) const;
    std::string summary() const;
};


#endif // CONFIGURATION_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
