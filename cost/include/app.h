#pragma once
#include <string>

class App {
public:
  App();
  void run();

private:
  std::string current_book_ = "default";

  // 账本管理文件
  const std::string books_index_ = "books.txt";
  const std::string current_book_file_ = "current_book.txt";

  void load_current_book();
  void save_current_book();
  void ensure_book_in_index(const std::string& book);
  void print_books();

  // UI 辅助
  static void press_enter();
  static std::string read_line(const std::string& prompt);
  static long long read_ll(const std::string& prompt);
  static double read_double(const std::string& prompt);
};
