#include "util.h"
#include <cctype>
#include <ctime>

std::string trim(const std::string& s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

bool is_valid_date(const std::string& d) {
  if (d.size() != 10) return false;
  if (d[4] != '-' || d[7] != '-') return false;
  for (int i = 0; i < 10; ++i) {
    if (i == 4 || i == 7) continue;
    if (!std::isdigit((unsigned char)d[i])) return false;
  }
  return true;
}

std::string today_date() {
  std::time_t now = std::time(nullptr);
  std::tm* lt = std::localtime(&now);
  char buf[11];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d", lt);
  return std::string(buf);
}

std::string month_of(const std::string& d) {
  if (d.size() >= 7) return d.substr(0, 7);
  return "";
}

std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == delim) {
      out.push_back(cur);
      cur.clear();
    } else cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

static std::string hex2(unsigned char x) {
  const char* h = "0123456789ABCDEF";
  std::string s;
  s.push_back(h[(x >> 4) & 0xF]);
  s.push_back(h[x & 0xF]);
  return s;
}

std::string pct_encode(const std::string& in) {
  std::string out;
  for (unsigned char c : in) {
    // 放行常见安全字符；其他都编码（确保日志分隔符不会破坏解析）
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == ' ') {
      out.push_back((char)c);
    } else {
      out.push_back('%');
      out += hex2(c);
    }
  }
  return out;
}

static int from_hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

std::string pct_decode(const std::string& in) {
  std::string out;
  for (size_t i = 0; i < in.size();) {
    if (in[i] == '%' && i + 2 < in.size()) {
      int hi = from_hex(in[i + 1]);
      int lo = from_hex(in[i + 2]);
      if (hi >= 0 && lo >= 0) {
        out.push_back((char)((hi << 4) | lo));
        i += 3;
        continue;
      }
    }
    out.push_back(in[i]);
    i++;
  }
  return out;
}
