#include <iostream>
#include <cstdlib> // For exit()

#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <iostream>
#include <string>
#include <stdexcept>

#include "configuration.h"


// Helper function to flatten a YAML node recursively into dot-separated keys.
namespace {
    void flattenYAML(const YAML::Node& node, const std::string& prefix, std::unordered_map<std::string, std::string>& result) {
        if (node.IsScalar()) {
            result[prefix] = node.as<std::string>();
        } else if (node.IsMap()) {
            for (auto it = node.begin(); it != node.end(); ++it) {
                std::string key = it->first.as<std::string>();
                std::string newPrefix = prefix.empty() ? key : prefix + "." + key;
                flattenYAML(it->second, newPrefix, result);
            }
        } else if (node.IsSequence()) {
            for (std::size_t i = 0; i < node.size(); i++) {
                std::string newPrefix = prefix + "[" + std::to_string(i) + "]";
                flattenYAML(node[i], newPrefix, result);
            }
        }
    }
}

void Configuration::load(const std::string& file_name) {
    try {
        YAML::Node yaml_config = YAML::LoadFile(file_name);
        // Clear any existing configuration
        config_map.clear();
        // Recursively flatten the YAML structure
        flattenYAML(yaml_config, "", config_map);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Error reading the YAML file: " + std::string(e.what()));
    }
}

std::string Configuration::get(const std::string& key, const std::string& default_value) const {
    auto it = config_map.find(key);
    if (it != config_map.end()) {
        return it->second;
    }
    return default_value;
}

void Configuration::set(const std::string& key, const std::string& value) {
    config_map[key] = value;
}

bool Configuration::contains(const std::string& key) const {
    return config_map.find(key) != config_map.end();
}

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
