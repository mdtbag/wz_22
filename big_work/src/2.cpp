#include <stdio.h>  
#include <stdlib.h>  
#include <math.h> 
#include "rapidcsv.h"
#define E 2.7182818284590452354 //自然常数
typedef struct {  
    double x1; //嘴峰长度 
    double x2;   //嘴峰深度 
    int label;//类别标签,0:Adelie,1:Gentoo 
} DataPoint;  
int ReadCsv(const char* fileName, DataPoint data[]) {
    rapidcsv::Document doc(fileName, rapidcsv::LabelParams(),
        rapidcsv::SeparatorParams(),rapidcsv::ConverterParams(true));//解析csv文件 
    int j=0;//样本计数 
    for (int i=0;i<doc.GetRowCount();i++){ //每次处理一行 
        double bill_length_mm = doc.GetCell<double>("bill_length_mm", i);//嘴峰长度 
        double bill_depth_mm = doc.GetCell<double>("bill_depth_mm", i);//嘴峰深度 
        std::string type_name = doc.GetCell<std::string>("species", i);//种族
        if ( (type_name=="Adelie" || type_name=="Gentoo") 
        && !std::isnan(bill_length_mm) && !std::isnan(bill_depth_mm)){//处理缺失值 
            data[j].x1 = bill_length_mm;
            data[j].x2 = bill_depth_mm;
            data[j].label = type_name == "Adelie"?0:1;
            j++;//剔除缺失值后的样本数目 
        }
    }
    return j; 
}  
//推理函数 
double Predict(double theta0, double theta1, double theta2, double x1, double x2){
    return  1.0/(1.0 + pow(E, -(theta0 + theta1 * x1 + theta2 * x2)));
}    
//模型训练函数 
void GradientDescent(DataPoint data[],int n,double alpha,double esp,int iterations,double*theta0,double*theta1,double*theta2) {  
    int iter;
    for (iter = 0; iter < iterations; iter++) { //迭代 
        double grad0 = 0, grad1 = 0, grad2 = 0;  //偏导数
        for (int i = 0; i < n; i++) { //遍历数据集 
            double predicted = Predict(*theta0, *theta1, *theta2, data[i].x1, data[i].x2);  
            double diff = predicted- data[i].label;
            grad0 += diff; grad1 += data[i].x1*diff; grad2 += data[i].x2*diff; //计算偏导数 
        } 
        printf("%d theta0: %f, theta1: %f, theta2: %f\n", iter+1, *theta0, *theta1, *theta2);       
        grad0/=n;  grad1/=n; grad2/=n;//计算偏导数
        *theta0 -= alpha * grad0; *theta1 -= alpha * grad1; *theta2 -= alpha * grad2; //更新参数 
        printf("    grad0: %f, grad1: %f, grad2: %f\n", grad0, grad1, grad2);       
        if (fabs(alpha*grad0)<=esp && fabs(alpha*grad1)<=esp && fabs(alpha*grad2)<=esp){ //判断是否达到精度要求 
            printf("Converged after %d iterations.\n", iter + 1);
            break;
        }
    }  
    if (iter>=iterations)printf("Reached maximum number of iterations without converging.\n");
}
int main() {  
    DataPoint data[500];
    int n = ReadCsv("penguins.csv", data);//从文件读入数据
    double theta0 = 0.5, theta1 = 0.5, theta2 = 0.5; // 初始参数值  
    double alpha = 0.001; // 学习率  
    double esp = 0.00001; //精度阈值 
    int iterations = 100000; // 迭代次数  
    GradientDescent(data, n, alpha, esp, iterations, &theta0, &theta1, &theta2);//训练模型  
    printf("theta0: %f, theta1: %f, theta2: %f\n", theta0, theta1, theta2);       
    double output = Predict(theta0,theta1,theta2, 46.2, 13.5);//用模型进行推理 
    printf("The output is %lf\n", output);
    return 0;  
}
