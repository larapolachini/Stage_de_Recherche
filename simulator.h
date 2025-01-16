#ifndef SIMULATOR_H
#define SIMULATOR_H

//#include "spogobot.h"
#include "pogosim.h"

extern "C" int robot_main(void);

class Configuration {
private:
    std::unordered_map<std::string, std::string> config_map;
public:
    void load(const std::string& file_name);
    std::string get(const std::string& key, const std::string& default_value = "") const;
    bool contains(const std::string& key) const;
    std::string summary() const;
};

void create_robots(Configuration& config);
void main_loop(Configuration& config);
void set_current_robot(Robot& robot);

#endif // SIMULATOR_H

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
