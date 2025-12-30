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
int main() {  
    char model_file_name[] = "svmmod.xml"; 
    Ptr<SVM> svm = Algorithm::load<SVM>(model_file_name);  //从文件加载训练好的SVM模型 
    float new_samp_data[] = {51.2,18.7,192,4250,1,0};//新的企鹅数据样本
    Mat new_sample(1, 6, CV_32F, new_samp_data); //将新样本数据填充到Mat对象中  
    float response = svm->predict(new_sample); // 进行预测  
    cout << "Predicted species: " << response << endl; // 输出预测结果 
    return 0;  
}
