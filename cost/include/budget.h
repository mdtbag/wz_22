#pragma once
#include <string>

void set_budget(const std::string& book,
                const std::string& ym,
                double amount);

bool get_budget(const std::string& book,
                const std::string& ym,
                double& amount);
