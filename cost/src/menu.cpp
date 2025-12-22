#include "menu.h"

#include "query.h"
#include "chart.h"
#include "import.h"
#include "util.h"

#include <iostream>
#include <unordered_map>
#include <limits>
#include <fstream>

// ================= 私有工具函数（仅 menu.cpp 可见） =================

// 本地读取一行输入（不依赖 App）
static std::string read_line_local(const std::string& prompt) {
    std::cout << prompt;
    std::cout.flush();
    std::string s;
    std::getline(std::cin, s);
    return s;
}

// ================= 9) 时间段查询 =================

void menu_query_range(Ledger& ledger) {
    std::string a = trim(read_line_local("开始日期 YYYY-MM-DD："));
    std::string b = trim(read_line_local("结束日期 YYYY-MM-DD："));

    if (!is_valid_date(a) || !is_valid_date(b) || a > b) {
        std::cout << "❌ 日期非法\n";
        return;
    }

    auto all = ledger.list_sorted();
    auto v = filter_by_range(all, a, b);

    if (v.empty()) {
        std::cout << "（无记录）\n";
        return;
    }

    for (auto& r : v) {
        std::cout << r.id << " "
                  << r.date << " "
                  << r.platform << " "
                  << r.amount << " "
                  << r.note << "\n";
    }

    std::cout << "合计：" << sum_amount(v) << "\n";
}

// ================= 10) 关键词搜索 =================

void menu_search_keyword(Ledger& ledger) {
    std::string kw = trim(read_line_local("关键词（平台 / 备注）："));
    if (kw.empty()) {
        std::cout << "❌ 关键词不能为空\n";
        return;
    }

    auto all = ledger.list_sorted();
    auto v = filter_by_keyword(all, kw);

    if (v.empty()) {
        std::cout << "（无匹配记录）\n";
        return;
    }

    for (auto& r : v) {
        std::cout << r.id << " "
                  << r.date << " "
                  << r.platform << " "
                  << r.amount << " "
                  << r.note << "\n";
    }

    std::cout << "合计：" << sum_amount(v) << "\n";
}

// ================= 11) 月度总支出图表 =================

void menu_chart_monthly(Ledger& ledger) {
    auto all = ledger.list_sorted();
    if (all.empty()) {
        std::cout << "（无数据）\n";
        return;
    }

    std::unordered_map<std::string, double> month_sum;
    for (auto& r : all) {
        month_sum[month_of(r.date)] += r.amount;
    }

    std::cout << "\n📊 月度总支出\n";
    draw_bar_chart(month_sum, 40, -1);
}

// ================= 12) 平台 Top-N =================

void menu_chart_platform_top(Ledger& ledger) {
    int n;
    std::cout << "Top N（如 5 / 10）：";
    std::cin >> n;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (n <= 0) {
        std::cout << "❌ N 必须大于 0\n";
        return;
    }

    auto all = ledger.list_sorted();
    auto mp = sum_by_platform(all);

    if (mp.empty()) {
        std::cout << "（无数据）\n";
        return;
    }

    std::cout << "\n📊 平台支出 Top " << n << "\n";
    draw_bar_chart(mp, 40, n);
}

// ================= 13) 从 import.txt 批量导入 =================

void menu_import_file(Ledger& ledger) {
    const std::string filename = "import.txt";

    std::ifstream test(filename);
    if (!test.is_open()) {
        std::cout << "❌ 未找到文件：" << filename << "\n";
        std::cout << "请在程序目录创建 import.txt\n";
        return;
    }
    test.close();

    std::string ans = trim(read_line_local("确认从 import.txt 导入？(y/N)："));
    if (ans.empty() || (ans[0] != 'y' && ans[0] != 'Y')) {
        std::cout << "已取消导入\n";
        return;
    }

    import_from_file(ledger, filename);
    std::cout << "✅ 导入完成\n";
}
