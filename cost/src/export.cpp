#include "export.h"
#include <fstream>

void export_csv(const Ledger& ledger,
                const std::string& filename){

    std::ofstream out(filename,std::ios::trunc);
    out<<"id,date,platform,amount,note\n";
    for(auto&r:ledger.list_sorted()){
        out<<r.id<<","<<r.date<<","<<r.platform
           <<","<<r.amount<<","<<r.note<<"\n";
    }
}
