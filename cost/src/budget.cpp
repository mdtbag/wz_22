#include "budget.h"
#include <fstream>
#include <unordered_map>

static std::string file_of(const std::string& book){
    return "budget_" + book + ".txt";
}

void set_budget(const std::string& book,
                const std::string& ym,
                double amount){
    std::unordered_map<std::string,double> m;
    std::ifstream in(file_of(book));
    std::string k; double v;
    while(in>>k>>v) m[k]=v;
    m[ym]=amount;

    std::ofstream out(file_of(book),std::ios::trunc);
    for(auto& kv:m)
        out<<kv.first<<" "<<kv.second<<"\n";
}

bool get_budget(const std::string& book,
                const std::string& ym,
                double& amount){
    std::ifstream in(file_of(book));
    std::string k; double v;
    while(in>>k>>v){
        if(k==ym){ amount=v; return true; }
    }
    return false;
}
