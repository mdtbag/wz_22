#include "chart.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>

void draw_bar_chart(
    const std::unordered_map<std::string,double>& m,
    int width,
    int topN) {

    if (m.empty()) {
        std::cout << "（无数据）\n";
        return;
    }

    std::vector<std::pair<std::string,double>> v(m.begin(), m.end());
    std::sort(v.begin(), v.end(),
              [](auto& a, auto& b){ return a.second > b.second; });

    if (topN > 0 && (int)v.size() > topN)
        v.resize(topN);

    double maxv = v.front().second;
    if (maxv <= 0) maxv = 1;

    for (auto& kv : v) {
        int bar = (int)(kv.second / maxv * width);
        std::cout << std::left << std::setw(14) << kv.first << " | ";
        for (int i = 0; i < bar; ++i) std::cout << "█";
        std::cout << " " << std::fixed << std::setprecision(2)
                  << kv.second << "\n";
    }
}
