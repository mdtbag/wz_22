#pragma once
#include <string>

struct Record {
  long long id = 0;
  std::string date;      // YYYY-MM-DD
  std::string platform;  // 项目/平台
  double amount = 0.0;
  std::string note;      // 备注（可空）
};
