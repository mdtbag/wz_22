#pragma once
#include <unordered_map>
#include <string>

void draw_bar_chart(
    const std::unordered_map<std::string,double>& m,
    int width = 40,
    int topN = -1);
