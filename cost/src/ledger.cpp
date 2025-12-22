#include "ledger.h"
#include "util.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

Ledger::Ledger(std::string book) : book_(std::move(book)) {}

std::string Ledger::record_file() const { return "record_" + book_ + ".txt"; }
std::string Ledger::log_file() const { return "oplog_" + book_ + ".txt"; }

void Ledger::ensure_file_exists(const std::string& path) {
  std::ofstream f(path, std::ios::app);
}

std::string Ledger::op_type_to_string(OpType t) {
  switch (t) {
    case OpType::ADD:  return "ADD";
    case OpType::DEL:  return "DEL";
    case OpType::EDIT: return "EDIT";
  }
  return "ADD";
}

bool Ledger::string_to_op_type(const std::string& s, OpType& t) {
  if (s == "ADD")  { t = OpType::ADD;  return true; }
  if (s == "DEL")  { t = OpType::DEL;  return true; }
  if (s == "EDIT") { t = OpType::EDIT; return true; }
  return false;
}

// 日志格式：TAG|type=ADD|after_id=...|after_date=...|after_platform=...|after_amount=...|after_note=...
//        或：TAG|type=DEL|before_...
//        或：TAG|type=EDIT|before_...|after_...
std::string Ledger::op_to_line(const std::string& tag, const Op& op) {
  auto rec_kv = [](const std::string& prefix, const Record& r) {
    std::ostringstream oss;
    oss << "|" << prefix << "id=" << r.id;
    oss << "|" << prefix << "date=" << pct_encode(r.date);
    oss << "|" << prefix << "platform=" << pct_encode(r.platform);
    oss << "|" << prefix << "amount=" << std::fixed << std::setprecision(2) << r.amount;
    oss << "|" << prefix << "note=" << pct_encode(r.note);
    return oss.str();
  };

  std::ostringstream oss;
  oss << tag << "|type=" << op_type_to_string(op.type);

  if (op.type == OpType::ADD) {
    oss << rec_kv("after_", op.after);
  } else if (op.type == OpType::DEL) {
    oss << rec_kv("before_", op.before);
  } else {
    oss << rec_kv("before_", op.before);
    oss << rec_kv("after_", op.after);
  }
  return oss.str();
}

bool Ledger::line_to_tag_op(const std::string& line, std::string& tag, Op& op) {
  auto parts = split(line, '|');
  if (parts.size() < 2) return false;
  tag = parts[0];

  std::unordered_map<std::string, std::string> kv;
  for (size_t i = 1; i < parts.size(); ++i) {
    auto pos = parts[i].find('=');
    if (pos == std::string::npos) continue;
    kv[parts[i].substr(0, pos)] = parts[i].substr(pos + 1);
  }

  auto it = kv.find("type");
  if (it == kv.end()) return false;
  if (!string_to_op_type(it->second, op.type)) return false;

  auto get_rec = [&](const std::string& prefix, Record& r) -> bool {
    auto gid = kv.find(prefix + "id");
    auto gd  = kv.find(prefix + "date");
    auto gp  = kv.find(prefix + "platform");
    auto ga  = kv.find(prefix + "amount");
    auto gn  = kv.find(prefix + "note");
    if (gid == kv.end() || gd == kv.end() || gp == kv.end() || ga == kv.end() || gn == kv.end()) return false;
    r.id = std::stoll(gid->second);
    r.date = pct_decode(gd->second);
    r.platform = pct_decode(gp->second);
    r.amount = std::stod(ga->second);
    r.note = pct_decode(gn->second);
    return true;
  };

  if (op.type == OpType::ADD) return get_rec("after_", op.after);
  if (op.type == OpType::DEL) return get_rec("before_", op.before);
  return get_rec("before_", op.before) && get_rec("after_", op.after);
}

void Ledger::append_log(const std::string& tag, const Op& op) const {
  ensure_file_exists(log_file());
  std::ofstream out(log_file(), std::ios::app);
  if (!out.is_open()) return;
  out << op_to_line(tag, op) << "\n";
  out.flush(); // 提高崩溃时的日志完整性
}

void Ledger::rewrite_record_file() const {
  ensure_file_exists(record_file());
  std::vector<Record> v;
  v.reserve(by_id_.size());
  for (auto& kv : by_id_) v.push_back(kv.second);

  std::sort(v.begin(), v.end(), [](const Record& a, const Record& b) {
    if (a.date != b.date) return a.date < b.date;
    return a.id < b.id;
  });

  std::ofstream out(record_file(), std::ios::trunc);
  if (!out.is_open()) return;

  for (auto& r : v) {
    out << r.id << " " << r.date << " " << r.platform << " " << std::fixed << std::setprecision(2) << r.amount;
    if (!r.note.empty()) out << " " << r.note;
    out << "\n";
  }
  out.flush();
}

void Ledger::apply_forward(const Op& op) {
  if (op.type == OpType::ADD) {
    by_id_[op.after.id] = op.after;
    next_id_ = std::max(next_id_, op.after.id + 1);
  } else if (op.type == OpType::DEL) {
    by_id_.erase(op.before.id);
  } else { // EDIT
    by_id_[op.after.id] = op.after;
    next_id_ = std::max(next_id_, op.after.id + 1);
  }
}

void Ledger::apply_inverse(const Op& op) {
  if (op.type == OpType::ADD) {
    by_id_.erase(op.after.id);
  } else if (op.type == OpType::DEL) {
    by_id_[op.before.id] = op.before;
    next_id_ = std::max(next_id_, op.before.id + 1);
  } else { // EDIT
    by_id_[op.before.id] = op.before;
    next_id_ = std::max(next_id_, op.before.id + 1);
  }
}

void Ledger::replay_log_build_stacks_only() {
  ensure_file_exists(log_file());
  std::ifstream in(log_file());
  std::string line;

  // 只建立撤销链（跨重启），不在这里二次修改 record
  // 真正修复：用 rebuild_from_log_and_rewrite_record()
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty()) continue;

    std::string tag;
    Op op;
    if (!line_to_tag_op(line, tag, op)) continue;

    if (tag == "DO") {
      undo_.push_back(op);
      redo_.clear();
    } else if (tag == "UNDO") {
      if (!undo_.empty()) {
        Op top = undo_.back();
        undo_.pop_back();
        redo_.push_back(top);
      }
    } else if (tag == "REDO") {
      if (!redo_.empty()) {
        Op top = redo_.back();
        redo_.pop_back();
        undo_.push_back(top);
      }
    }
  }
}

void Ledger::load() {
  if (loaded_) return;
  loaded_ = true;

  ensure_file_exists(record_file());
  ensure_file_exists(log_file());

  // 1) 先从 record 读当前状态（快速启动）
  {
    std::ifstream in(record_file());
    std::string line;
    long long max_id = 0;
    while (std::getline(in, line)) {
      line = trim(line);
      if (line.empty()) continue;

      std::istringstream iss(line);
      Record r;
      if (!(iss >> r.id >> r.date >> r.platform >> r.amount)) continue;
      std::string rest;
      std::getline(iss, rest);
      r.note = trim(rest);

      if (r.id <= 0 || !is_valid_date(r.date) || r.platform.empty()) continue;
      by_id_[r.id] = r;
      max_id = std::max(max_id, r.id);
    }
    next_id_ = max_id + 1;
  }

  // 2) 从日志建立 undo/redo 栈（支持跨重启撤销链）
  replay_log_build_stacks_only();
}

void Ledger::commit_new_op(const Op& op) {
  load();
  apply_forward(op);
  rewrite_record_file();

  undo_.push_back(op);
  redo_.clear();

  append_log("DO", op);
}

long long Ledger::add(const std::string& date, const std::string& platform, double amount, const std::string& note) {
  load();
  Record r;
  r.id = next_id_++;
  r.date = date;
  r.platform = platform;
  r.amount = amount;
  r.note = note;

  Op op;
  op.type = OpType::ADD;
  op.after = r;

  commit_new_op(op);
  return r.id;
}

bool Ledger::edit(long long id, const std::string& new_date, const std::string& new_platform,
                  const std::string& new_amount_opt, const std::string& new_note_opt) {
  load();
  auto it = by_id_.find(id);
  if (it == by_id_.end()) return false;

  Record before = it->second;
  Record after = before;

  if (!new_date.empty()) after.date = new_date;
  if (!new_platform.empty()) after.platform = new_platform;
  if (!new_amount_opt.empty()) {
    try { after.amount = std::stod(new_amount_opt); } catch (...) { return false; }
  }
  if (!new_note_opt.empty()) {
    if (new_note_opt == "-") after.note.clear();
    else after.note = new_note_opt;
  }

  // 基本校验
  if (!is_valid_date(after.date) || after.platform.empty()) return false;

  Op op;
  op.type = OpType::EDIT;
  op.before = before;
  op.after = after;

  commit_new_op(op);
  return true;
}

bool Ledger::remove(long long id) {
  load();
  auto it = by_id_.find(id);
  if (it == by_id_.end()) return false;

  Record before = it->second;

  Op op;
  op.type = OpType::DEL;
  op.before = before;

  commit_new_op(op);
  return true;
}

bool Ledger::undo() {
  load();
  if (undo_.empty()) return false;

  Op op = undo_.back();
  undo_.pop_back();

  apply_inverse(op);
  rewrite_record_file();

  redo_.push_back(op);
  append_log("UNDO", op);
  return true;
}

bool Ledger::redo() {
  load();
  if (redo_.empty()) return false;

  Op op = redo_.back();
  redo_.pop_back();

  apply_forward(op);
  rewrite_record_file();

  undo_.push_back(op);
  append_log("REDO", op);
  return true;
}

std::vector<Record> Ledger::list_sorted() const {
  std::vector<Record> v;
  v.reserve(by_id_.size());
  for (auto& kv : by_id_) v.push_back(kv.second);
  std::sort(v.begin(), v.end(), [](const Record& a, const Record& b) {
    if (a.date != b.date) return a.date < b.date;
    return a.id < b.id;
  });
  return v;
}

bool Ledger::has_id(long long id) const {
  return by_id_.find(id) != by_id_.end();
}

Record Ledger::get(long long id) const {
  auto it = by_id_.find(id);
  if (it == by_id_.end()) return Record{};
  return it->second;
}

void Ledger::rebuild_from_log_and_rewrite_record() {
  ensure_file_exists(log_file());

  // 从空状态开始，以日志为真相源重建（这是“修复/崩溃恢复”的上限做法）
  std::unordered_map<long long, Record> state;
  std::vector<Op> u, r;
  long long max_id = 0;

  auto apply_forward_state = [&](const Op& op) {
    if (op.type == OpType::ADD) {
      state[op.after.id] = op.after;
      max_id = std::max(max_id, op.after.id);
    } else if (op.type == OpType::DEL) {
      state.erase(op.before.id);
    } else {
      state[op.after.id] = op.after;
      max_id = std::max(max_id, op.after.id);
    }
  };
  auto apply_inverse_state = [&](const Op& op) {
    if (op.type == OpType::ADD) {
      state.erase(op.after.id);
    } else if (op.type == OpType::DEL) {
      state[op.before.id] = op.before;
      max_id = std::max(max_id, op.before.id);
    } else {
      state[op.before.id] = op.before;
      max_id = std::max(max_id, op.before.id);
    }
  };

  std::ifstream in(log_file());
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty()) continue;

    std::string tag;
    Op op;
    if (!line_to_tag_op(line, tag, op)) continue;

    if (tag == "DO") {
      apply_forward_state(op);
      u.push_back(op);
      r.clear();
    } else if (tag == "UNDO") {
      if (!u.empty()) {
        Op top = u.back(); u.pop_back();
        apply_inverse_state(top);
        r.push_back(top);
      }
    } else if (tag == "REDO") {
      if (!r.empty()) {
        Op top = r.back(); r.pop_back();
        apply_forward_state(top);
        u.push_back(top);
      }
    }
  }

  // 用重建结果替换当前内存并重写 record
  by_id_ = std::move(state);
  undo_ = std::move(u);
  redo_ = std::move(r);
  next_id_ = max_id + 1;

  rewrite_record_file();
}
