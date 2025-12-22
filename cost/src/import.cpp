#include "import.h"
#include "command.h"
#include "util.h"
#include <fstream>
#include <sstream>
#include <iostream>

void import_from_file(Ledger& ledger,
                      const std::string& filename){

    std::ifstream in(filename);
    if(!in.is_open()){
        std::cout<<"❌ 无法打开 "<<filename<<"\n";
        return;
    }

    int ok=0, skip=0;
    std::string line;
    while(getline(in,line)){
        line=trim(line);
        if(line.empty()||line[0]=='#') continue;

        std::string p,n;
        double a;

        if(try_quick_command(line,p,a,n)){
            ledger.add(today_date(),p,a,n);
            ok++; continue;
        }

        std::istringstream iss(line);
        std::string first;
        iss>>first;
        if(is_valid_date(first)){
            std::string plat; double amt;
            iss>>plat>>amt;
            getline(iss,n);
            ledger.add(first,plat,amt,trim(n));
            ok++;
        }else{
            double amt;
            iss>>amt;
            getline(iss,n);
            ledger.add(today_date(),first,amt,trim(n));
            ok++;
        }
    }
    std::cout<<"✅ 导入完成："<<ok<<" 条\n";
}
