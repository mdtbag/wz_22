#pragma once
#include "record.h"
#include <string>
#include <unordered_map>
#include <vector>
#include <iomanip>

enum class OpType { ADD, DEL, EDIT };

struct Op {
  OpType type{};
  Record before; // DEL/EDIT
  Record after;  // ADD/EDIT
};

class Ledger {
public:
  explicit Ledger(std::string book);

  // 账本名 & 文件名
  const std::string& book_name() const { return book_; }
  std::string record_file() const;
  std::string log_file() const;

  // 启动加载：从 record + oplog 建立内存状态与 undo/redo（跨重启）
  void load();

  // 修复/恢复：完全以 oplog 为真相源，重建并重写 record（崩溃/不一致时用）
  void rebuild_from_log_and_rewrite_record();

  // 基本 CRUD（都写日志，可 Undo/Redo）
  long long add(const std::string& date, const std::string& platform, double amount, const std::string& note);
  bool edit(long long id, const std::string& new_date, const std::string& new_platform,
            const std::string& new_amount_opt, const std::string& new_note_opt);
  bool remove(long long id);

  // Undo/Redo
  bool undo();
  bool redo();

  // 查询/列表（用于验证）
  std::vector<Record> list_sorted() const;
  bool has_id(long long id) const;
  Record get(long long id) const;

  // 状态
  size_t undo_size() const { return undo_.size(); }
  size_t redo_size() const { return redo_.size(); }

private:
  std::string book_;
  bool loaded_ = false;

  std::unordered_map<long long, Record> by_id_;
  long long next_id_ = 1;

  std::vector<Op> undo_;
  std::vector<Op> redo_;

  // 文件辅助
  static void ensure_file_exists(const std::string& path);

  // record 持久化（把当前内存写成最终态）
  void rewrite_record_file() const;

  // 日志 I/O（持久化操作链）
  void append_log(const std::string& tag, const Op& op) const;

  // 解析/序列化 op
  static std::string op_to_line(const std::string& tag, const Op& op);
  static bool line_to_tag_op(const std::string& line, std::string& tag, Op& op);

  // 执行 op（内存层）
  void apply_forward(const Op& op);
  void apply_inverse(const Op& op);

  // 从日志建立 undo/redo 栈（跨重启）
  void replay_log_build_stacks_only();

  // 在用户“新操作”时：写日志、更新栈、写 record
  void commit_new_op(const Op& op);

  static std::string op_type_to_string(OpType t);
  static bool string_to_op_type(const std::string& s, OpType& t);
};
