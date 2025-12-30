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
    Mat train_labels(n1, 3, CV_32F);//训练数据的标签（独热编码） 
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
            if (data[i].label==0) {//标签转换为独热编码 
               train_labels.at<float>(k, 0)=1;train_labels.at<float>(k, 1)=0;train_labels.at<float>(k, 2)=0;
            }else if (data[i].label==1) {
               train_labels.at<float>(k, 0)=0;train_labels.at<float>(k, 1)=1;train_labels.at<float>(k, 2)=0;
            }else{
               train_labels.at<float>(k, 0)=0;train_labels.at<float>(k, 1)=0;train_labels.at<float>(k, 2)=1;
            }
            k++;
        }
    }    
    cout<<"n="<<n<<",n1="<<n1<<",n2="<<n2<<endl;
    Ptr<ANN_MLP> nn = ANN_MLP::create(); //创建神经网络模型
    nn->setLayerSizes(vector<int>{6, 10, 3});//输入层6个节点，隐藏层10个节点，输出层3个节点（假设3类企鹅）  
    nn->setActivationFunction(ANN_MLP::SIGMOID_SYM);//激活函数目前只支持Sigmoid 
    nn->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 10000, 1e-5));//设置迭代次数与收敛条件  
    nn->setTrainMethod(ANN_MLP::RPROP); //弹性反向传播算法  
    nn->train(train_samples, ROW_SAMPLE, train_labels);//训练模型
    Mat predictions; //预测结果 
    nn->predict(test_samples,predictions);//对测试集进行预测  
    int correct = 0;  //正确分类的次数
    for (int i = 0; i < predictions.rows; i++){        
        Point max_loc; // 找到预测值中的最大索引，即预测的类别   
        minMaxLoc(predictions.row(i), nullptr, nullptr, nullptr, &max_loc);  
        cout<<"predictions:"<<predictions.row(i)<<", max_loc.x:"<<max_loc.x<<", label:"<<test_labels.at<int>(i, 0)<<endl;
        if (max_loc.x == test_labels.at<int>(i, 0)) correct++;   
    }
    double accuracy = (double)correct / predictions.rows; //计算准确率 
    cout<<"Accuracy: " << accuracy * 100 << "%" << endl;     
    return 0;  
}
