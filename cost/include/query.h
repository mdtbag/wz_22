#pragma once
#include "record.h"
#include <vector>
#include <string>
#include <unordered_map>

std::vector<Record> filter_by_date(
    const std::vector<Record>& all,
    const std::string& date);

std::vector<Record> filter_by_month(
    const std::vector<Record>& all,
    const std::string& ym);

std::vector<Record> filter_by_range(
    const std::vector<Record>& all,
    const std::string& from,
    const std::string& to);

std::vector<Record> filter_by_platform(
    const std::vector<Record>& all,
    const std::string& platform);

std::vector<Record> filter_by_keyword(
    const std::vector<Record>& all,
    const std::string& keyword);

double sum_amount(const std::vector<Record>& v);

std::unordered_map<std::string,double>
sum_by_platform(const std::vector<Record>& v);
