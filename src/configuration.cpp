#include <iostream>
#include <cstdlib> // For exit()

#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <iostream>
#include <string>

#include "configuration.h"


// Load configuration from a YAML file
void Configuration::load(const std::string& file_name) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(file_name);

        for (const auto& item : yaml_config) {
            std::string key = item.first.as<std::string>();
            std::string value = item.second.as<std::string>();
            config_map[key] = value;
        }
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Error reading the YAML file: " + std::string(e.what()));
    }
}

// Get a configuration value by key
std::string Configuration::get(const std::string& key, const std::string& default_value) const {
    auto it = config_map.find(key);
    if (it != config_map.end()) {
        return it->second;
    }
    return default_value;
}

// Check if a key exists in the configuration
bool Configuration::contains(const std::string& key) const {
    return config_map.find(key) != config_map.end();
}

// Return all configuration parameters
std::string Configuration::summary() const {
    std::ostringstream oss;
    oss << "Configuration Parameters:\n";
    for (const auto& [key, value] : config_map) {
        oss << key << ": " << value << "\n";
    }
    return oss.str();
}

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
