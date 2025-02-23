#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "pogosim.h"

/**
 * @brief Class for managing configuration parameters.
 *
 * The Configuration class provides methods to load configuration parameters from a YAML file,
 * access individual configuration values, modify configurations, and generate a summary of all parameters.
 */
class Configuration {
private:
    /// Internal storage for configuration key-value pairs.
    std::unordered_map<std::string, std::string> config_map;
public:
    /**
     * @brief Loads configuration parameters from a YAML file.
     *
     * Reads the YAML file specified by @a file_name and populates the internal configuration map
     * with key-value pairs. If an error occurs during file reading or parsing, a std::runtime_error is thrown.
     *
     * @param file_name The path to the YAML configuration file.
     *
     * @throws std::runtime_error if the YAML file cannot be read or parsed.
     */
    void load(const std::string& file_name);

    /**
     * @brief Retrieves the configuration value for a given key.
     *
     * Returns the value associated with the specified @a key. If the key is not present, the provided
     * @a default_value is returned.
     *
     * @param key The configuration key to look up.
     * @param default_value The value to return if the key is not found. Defaults to an empty string.
     * @return std::string The value corresponding to the key, or @a default_value if not found.
     */
    std::string get(const std::string& key, const std::string& default_value = "") const;

    /**
     * @brief Sets a configuration parameter.
     *
     * Assigns the specified @a value to the given configuration @a key.
     *
     * @param key The configuration key.
     * @param value The value to associate with the key.
     */
    void set(std::string const& key, std::string const& value);

    /**
     * @brief Checks if a configuration key exists.
     *
     * Determines whether the internal configuration map contains the specified @a key.
     *
     * @param key The configuration key to check.
     * @return true if the key exists; false otherwise.
     */
    bool contains(const std::string& key) const;

    /**
     * @brief Generates a summary of all configuration parameters.
     *
     * Creates a human-readable string listing all key-value pairs stored in the configuration.
     *
     * @return std::string A summary of the configuration parameters.
     */
    std::string summary() const;
};


#endif // CONFIGURATION_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
