#include <iostream>
#include "rapidcsv.h"
#include <opencv2/opencv.hpp>  
using namespace std;  
using namespace cv;  
using namespace cv::ml;  
typedef struct {  
    double x1,x2,x3,x4; //嘴峰长度、嘴峰深度、脚掌长度、 体重
    int x5_0,x5_1;  //是否male(1:male,0:female)；是否famale(1:female,0:male)
    int label; //类别标签, 0:Adelie,1:Gentoo,2: Chinstrap
} DataPoint;  
int ReadCsv(const char* fileName, DataPoint data[]) {
    rapidcsv::Document doc(fileName, rapidcsv::LabelParams(),
        rapidcsv::SeparatorParams(),rapidcsv::ConverterParams(true));//解析csv文件 
    int j=0;//样本计数 
    for (int i=0;i<doc.GetRowCount();i++){ //每次处理一行 
        double bill_length_mm = doc.GetCell<double>("bill_length_mm", i);//嘴峰长度 
        double bill_depth_mm = doc.GetCell<double>("bill_depth_mm", i);//嘴峰深度 
        double flipper_length_mm = doc.GetCell<double>("flipper_length_mm", i);//脚掌长度 
        double body_mass_g = doc.GetCell<double>("body_mass_g", i);//体重 
        std::string sex = doc.GetCell<std::string>("sex", i); //性别 
        std::string type_name = doc.GetCell<std::string>("species", i); //种族 
        if ( !std::isnan(bill_length_mm) && !std::isnan(bill_depth_mm)
        && !std::isnan(flipper_length_mm) && !std::isnan(body_mass_g) && sex!="NA" ){//处理缺失值 
            data[j].x1 = bill_length_mm;data[j].x2 = bill_depth_mm;
            data[j].x3 = flipper_length_mm;data[j].x4 = body_mass_g;
            data[j].x5_0 = sex=="male"?1:0;  data[j].x5_1 = sex=="female"?1:0;
            if (type_name == "Adelie") data[j].label=0;
            else if (type_name == "Gentoo") data[j].label=1;
            else data[j].label=2;
            j++;//剔除缺失值后的样本数目 
        }
    }
    return j; 
}   
int main() {  
    DataPoint data[500];
    int n = ReadCsv("penguins.csv", data);//从文件读入数据
    //将数据拷贝到Mat矩阵中    , 75%训练数据，25%测试数据 
    int n1=n*3/4, n2=n-n1;
    Mat train_samples(n1, 6, CV_32F); //训练数据集 
    Mat train_labels(n1, 1, CV_32S);//训练数据的标签 
    Mat test_samples(n2, 6, CV_32F); //测试数据集 
    Mat test_labels(n2, 1, CV_32S);//测试数据的标签 
    int i=0, j=0, k=0;
    for(i = 0; i<n; i++){
        if (i%4==0){//每3个训练数据搭配1个测试数据 
            test_samples.at<float>(j, 0) = data[i].x1;test_samples.at<float>(j, 1) = data[i].x2;
            test_samples.at<float>(j, 2) = data[i].x3;test_samples.at<float>(j, 3) = data[i].x4;
            test_samples.at<float>(j, 4) = data[i].x5_0;test_samples.at<float>(j, 5) = data[i].x5_1;
            test_labels.at<int>(j, 0) = data[i].label; j++;
        }else{
            train_samples.at<float>(k, 0) = data[i].x1;train_samples.at<float>(k, 1) = data[i].x2;
            train_samples.at<float>(k, 2) = data[i].x3;train_samples.at<float>(k, 3) = data[i].x4;
            train_samples.at<float>(k, 4) = data[i].x5_0;train_samples.at<float>(k, 5) = data[i].x5_1;
            train_labels.at<int>(k, 0) = data[i].label; k++;
        }
    }    
    cout<<"n="<<n<<",n1="<<n1<<",n2="<<n2<<endl;
    Ptr<SVM> svm = SVM::create(); //创建SVM模型 
    svm->setKernel(SVM::RBF); // 设置核函数RBF
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 10000, 1e-5));//设置迭代次数与收敛条件          
    double best_C, best_GAMMA, best_accuracy=0;     
    for (double C = 1e-2; C<=1e10; C*=10 ){
        for (double GAMMA = 1e-9; GAMMA<=1e3; GAMMA*=10 ){
            svm->setC(C); //惩罚系数C 
            svm->setGamma(GAMMA); //核函数的gamma参数 
            svm->train(train_samples, ROW_SAMPLE, train_labels);  // 训练SVM模型
            Mat predictions; //预测结果     
            svm->predict(test_samples,predictions);//对测试集进行预测  
            int correct = 0;  
            for (int i = 0; i < predictions.rows; i++)
                if (predictions.at<float>(i, 0) == test_labels.at<int>(i, 0)) correct++;//统计正确分类的次数 
            double accuracy = (double)correct / predictions.rows; //计算准确率 
            cout << "C: " << C << ", Gamma: " << GAMMA <<", Accuracy: " << accuracy * 100 << "%" << endl;  
            if (accuracy>best_accuracy){//本次参数组合优于最佳组合 
                best_C=C; best_GAMMA=GAMMA; best_accuracy=accuracy;
                svm->save("svmmod.xml");//存储模型 
            }
        }
    }
    cout<<"Best C: "<<best_C<<", Best Gamma: "<<best_GAMMA<<", Best Accuracy: "<<best_accuracy*100<<"%"<<endl;
    return 0;  
}
