#include "query.h"
#include "util.h"
#include <algorithm>

static bool in_range(const std::string& d,
                     const std::string& a,
                     const std::string& b) {
    return d >= a && d <= b;
}

std::vector<Record> filter_by_date(
    const std::vector<Record>& all,
    const std::string& date) {

    std::vector<Record> r;
    for (auto& x : all)
        if (x.date == date) r.push_back(x);
    return r;
}

std::vector<Record> filter_by_month(
    const std::vector<Record>& all,
    const std::string& ym) {

    std::vector<Record> r;
    for (auto& x : all)
        if (month_of(x.date) == ym) r.push_back(x);
    return r;
}

std::vector<Record> filter_by_range(
    const std::vector<Record>& all,
    const std::string& from,
    const std::string& to) {

    std::vector<Record> r;
    for (auto& x : all)
        if (in_range(x.date, from, to)) r.push_back(x);
    return r;
}

std::vector<Record> filter_by_platform(
    const std::vector<Record>& all,
    const std::string& platform) {

    std::vector<Record> r;
    for (auto& x : all)
        if (x.platform == platform) r.push_back(x);
    return r;
}

std::vector<Record> filter_by_keyword(
    const std::vector<Record>& all,
    const std::string& kw) {

    std::vector<Record> r;
    for (auto& x : all) {
        if (x.platform.find(kw) != std::string::npos ||
            x.note.find(kw) != std::string::npos)
            r.push_back(x);
    }
    return r;
}

double sum_amount(const std::vector<Record>& v) {
    double s = 0;
    for (auto& x : v) s += x.amount;
    return s;
}

std::unordered_map<std::string,double>
sum_by_platform(const std::vector<Record>& v) {
    std::unordered_map<std::string,double> m;
    for (auto& x : v) m[x.platform] += x.amount;
    return m;
}
