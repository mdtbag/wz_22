#include "command.h"
#include <sstream>

bool try_quick_command(
    const std::string& line,
    std::string& platform,
    double& amount,
    std::string& note) {

    static std::unordered_map<std::string,std::string> mp = {
        {"fs","食堂"}, {"jd","京东"}, {"mt","美团"}, {"tb","淘宝"}
    };

    std::istringstream iss(line);
    std::string key;
    if (!(iss >> key)) return false;
    if (mp.find(key) == mp.end()) return false;

    if (!(iss >> amount)) return false;
    getline(iss, note);
    if (!note.empty() && note[0]==' ') note.erase(0,1);

    platform = mp[key];
    return true;
}
