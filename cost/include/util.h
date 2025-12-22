#pragma once
#include <string>
#include <vector>

std::string trim(const std::string& s);
bool is_valid_date(const std::string& d);
std::string today_date();
std::string month_of(const std::string& d);

std::vector<std::string> split(const std::string& s, char delim);

// 为了日志稳定可解析：对 platform/note 做百分号编码（跨平台、可含中文/空格/符号）
std::string pct_encode(const std::string& in);
std::string pct_decode(const std::string& in);
