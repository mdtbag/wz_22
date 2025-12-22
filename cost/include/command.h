#pragma once
#include <string>
#include <unordered_map>

bool try_quick_command(
    const std::string& line,
    std::string& platform,
    double& amount,
    std::string& note);
