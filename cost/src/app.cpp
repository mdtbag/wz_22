#include "app.h"
#include "ledger.h"
#include "util.h"
#include "menu.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

// ================= 构造与账本管理 =================

App::App() {
    load_current_book();
    ensure_book_in_index(current_book_);
    save_current_book();
}

void App::load_current_book() {
    std::ifstream in(current_book_file_);
    if (!in.good()) return;
    std::string s;
    if (std::getline(in, s)) {
        s = trim(s);
        if (!s.empty()) current_book_ = s;
    }
}

void App::save_current_book() {
    std::ofstream out(current_book_file_, std::ios::trunc);
    if (out.is_open()) out << current_book_ << "\n";
}

void App::ensure_book_in_index(const std::string& book) {
    std::vector<std::string> v;
    {
        std::ifstream in(books_index_);
        std::string line;
        while (std::getline(in, line)) {
            line = trim(line);
            if (!line.empty()) v.push_back(line);
        }
    }
    if (std::find(v.begin(), v.end(), book) != v.end()) return;

    std::ofstream out(books_index_, std::ios::app);
    if (out.is_open()) out << book << "\n";
}

void App::print_books() {
    std::vector<std::string> v;
    std::ifstream in(books_index_);
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (!line.empty()) v.push_back(line);
    }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());

    std::cout << "已有账本：";
    if (v.empty()) std::cout << "（无）";
    for (auto& b : v) std::cout << " " << b;
    std::cout << "\n";
}

// ================= UI 辅助 =================

void App::press_enter() {
    std::cout << "按回车继续...";
    std::cout.flush();
    std::string tmp;
    std::getline(std::cin, tmp);
}

std::string App::read_line(const std::string& prompt) {
    std::cout << prompt;
    std::cout.flush();
    std::string s;
    std::getline(std::cin, s);
    return s;
}

long long App::read_ll(const std::string& prompt) {
    while (true) {
        std::string s = trim(read_line(prompt));
        if (s.empty()) {
            std::cout << "❌ 不能为空\n";
            continue;
        }
        try {
            size_t idx = 0;
            long long v = std::stoll(s, &idx);
            if (idx != s.size()) throw 1;
            return v;
        } catch (...) {
            std::cout << "❌ 请输入合法整数\n";
        }
    }
}

double App::read_double(const std::string& prompt) {
    while (true) {
        std::string s = trim(read_line(prompt));
        if (s.empty()) {
            std::cout << "❌ 不能为空\n";
            continue;
        }
        try {
            size_t idx = 0;
            double v = std::stod(s, &idx);
            if (idx != s.size()) throw 1;
            return v;
        } catch (...) {
            std::cout << "❌ 请输入合法数字\n";
        }
    }
}

// ================= 主循环 =================

void App::run() {
    while (true) {
        Ledger ledger(current_book_);
        ledger.load();

        std::cout << "\n========== 💳 记账本（最终版） ==========\n";
        std::cout << "当前账本: [" << current_book_ << "]\n";
        std::cout << "record: " << ledger.record_file() << "\n";
        std::cout << "oplog : " << ledger.log_file() << "\n";
        std::cout << "UNDO=" << ledger.undo_size()
                  << "  REDO=" << ledger.redo_size() << "\n";
        std::cout << "----------------------------------------\n";
        std::cout << "1) 添加\n";
        std::cout << "2) 编辑(按ID)\n";
        std::cout << "3) 删除(按ID)\n";
        std::cout << "4) 列表(按日期+ID排序)\n";
        std::cout << "5) Undo 撤销\n";
        std::cout << "6) Redo 重做\n";
        std::cout << "7) 切换账本\n";
        std::cout << "8) 修复/恢复：从日志重建 record\n";
        std::cout << "9) 查询：时间段\n";
        std::cout << "10) 搜索（关键词）\n";
        std::cout << "11) 图表：月度总支出\n";
        std::cout << "12) 图表：平台 Top-N\n";
        std::cout << "13) 从 import.txt 批量导入\n";
        std::cout << "0) 退出\n";
        std::cout << "请选择：";
        std::cout.flush();

        std::string c;
        std::getline(std::cin, c);
        c = trim(c);

        // ===== 退出 =====
        if (c == "0") {
            std::cout << "再见！\n";
            return;
        }

        // ===== 添加 =====
        if (c == "1") {
            std::string d = trim(read_line("日期(回车=今天；YYYY-MM-DD)："));
            if (d.empty()) d = today_date();
            if (!is_valid_date(d)) {
                std::cout << "❌ 日期格式错误\n";
                press_enter();
                continue;
            }

            std::string p = trim(read_line("平台/项目："));
            if (p.empty()) {
                std::cout << "❌ 不能为空\n";
                press_enter();
                continue;
            }

            double amt = read_double("金额：");
            std::string note = trim(read_line("备注(可空)："));

            long long id = ledger.add(d, p, amt, note);
            std::cout << "✅ 已添加 ID=" << id << "\n";
            press_enter();
            continue;
        }

        // ===== 编辑 =====
        if (c == "2") {
            long long id = read_ll("要编辑的ID：");
            if (!ledger.has_id(id)) {
                std::cout << "❌ 未找到该ID\n";
                press_enter();
                continue;
            }

            Record cur = ledger.get(id);
            std::cout << "当前："
                      << cur.id << " "
                      << cur.date << " "
                      << cur.platform << " "
                      << cur.amount << " "
                      << cur.note << "\n";

            std::string nd = trim(read_line("新日期(回车=不改)："));
            if (!nd.empty() && !is_valid_date(nd)) {
                std::cout << "❌ 日期格式错误\n";
                press_enter();
                continue;
            }

            std::string np = trim(read_line("新平台(回车=不改)："));
            std::string na = trim(read_line("新金额(回车=不改)："));
            std::string nn = trim(read_line("新备注(回车=不改；输入-清空)："));

            if (!ledger.edit(id, nd, np, na, nn))
                std::cout << "❌ 编辑失败\n";
            else
                std::cout << "✅ 已更新\n";

            press_enter();
            continue;
        }

        // ===== 删除 =====
        if (c == "3") {
            long long id = read_ll("要删除的ID：");
            if (!ledger.has_id(id)) {
                std::cout << "❌ 未找到该ID\n";
                press_enter();
                continue;
            }

            Record cur = ledger.get(id);
            std::cout << "将删除："
                      << cur.id << " "
                      << cur.date << " "
                      << cur.platform << " "
                      << cur.amount << " "
                      << cur.note << "\n";

            std::string ans = trim(read_line("确认删除？(y/N)："));
            if (!ans.empty() && (ans[0] == 'y' || ans[0] == 'Y')) {
                ledger.remove(id);
                std::cout << "✅ 已删除\n";
            } else {
                std::cout << "已取消\n";
            }
            press_enter();
            continue;
        }

        // ===== 列表 =====
        if (c == "4") {
            auto v = ledger.list_sorted();
            if (v.empty()) {
                std::cout << "（无记录）\n";
            } else {
                for (auto& r : v) {
                    std::cout << r.id << " "
                              << r.date << " "
                              << r.platform << " "
                              << r.amount << " "
                              << r.note << "\n";
                }
            }
            press_enter();
            continue;
        }

        // ===== Undo / Redo =====
        if (c == "5") {
            std::cout << (ledger.undo() ? "✅ 已撤销\n" : "ℹ️ 无可撤销操作\n");
            press_enter();
            continue;
        }

        if (c == "6") {
            std::cout << (ledger.redo() ? "✅ 已重做\n" : "ℹ️ 无可重做操作\n");
            press_enter();
            continue;
        }

        // ===== 切换账本 =====
        if (c == "7") {
            print_books();
            std::string nb = trim(read_line("输入账本名："));
            if (!nb.empty()) {
                current_book_ = nb;
                ensure_book_in_index(current_book_);
                save_current_book();
                std::cout << "✅ 已切换到账本 [" << current_book_ << "]\n";
            }
            press_enter();
            continue;
        }

        // ===== 修复 =====
        if (c == "8") {
            std::string ans = trim(read_line("确认从日志重建？(y/N)："));
            if (!ans.empty() && (ans[0] == 'y' || ans[0] == 'Y')) {
                ledger.rebuild_from_log_and_rewrite_record();
                std::cout << "✅ 修复完成\n";
            }
            press_enter();
            continue;
        }

        // ===== 查询 / 图表（统一函数）=====
        if (c == "9")  { menu_query_range(ledger);        press_enter(); continue; }
        if (c == "10") { menu_search_keyword(ledger);     press_enter(); continue; }
        if (c == "11") { menu_chart_monthly(ledger);      press_enter(); continue; }
        if (c == "12") { menu_chart_platform_top(ledger); press_enter(); continue; }
        if (c == "13") {
          menu_import_file(ledger);
          press_enter();
          continue;
        }

        // ===== 无效 =====
        std::cout << "❌ 无效选择\n";
        press_enter();
    }
}
