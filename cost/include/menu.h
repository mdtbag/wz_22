#pragma once

#include "ledger.h"

// 9) 时间段查询
void menu_query_range(Ledger& ledger);

// 10) 关键词搜索（平台 / 备注）
void menu_search_keyword(Ledger& ledger);

// 11) 图表：月度总支出
void menu_chart_monthly(Ledger& ledger);

// 12) 图表：平台 Top-N
void menu_chart_platform_top(Ledger& ledger);

// 13) 从 import.txt 导入
void menu_import_file(Ledger& ledger);
