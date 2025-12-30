#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <math.h> 
typedef struct {  
    double time_study; //ѧϰʱ�� 
    double marks;   //�ɼ� 
} DataPoint;  
int ReadCsv(const char* fileName, DataPoint data[]) {
    FILE *file;
    char line[100]; //�洢һ�����ݵ����� 
    int i = 0; //�����±� 
    file = fopen(fileName, "r"); //���ļ� 
    if (file == NULL) return -1; 
    fgets(line, sizeof(line), file); //���������� 
    while (fgets(line, sizeof(line), file) != NULL) {
        char *p1, *p2, *end;
        p1 = strchr(line, ','); //��1������λ�� 
        *p1=0; p1++;
        p2 = strchr(p1, ','); //��2������λ��
        *p2=0; p2++;
        data[i].time_study = strtod(p1, &end); //����time_study
        data[i].marks = strtod(p2, &end); //����marks
        i++; 
    }
    fclose(file); //�ر��ļ� 
    return i; //������������ 
}  
//�������� 
double Predict(double theta0, double theta1, double x){
    return theta0 + theta1*x;
}    
//ģ��ѵ������ 
void GradientDescent(DataPoint data[], int n, double alpha, double esp,int iterations, double *theta0, double *theta1) {  
    int iter;
    for (iter = 0; iter < iterations; iter++) { //���� 
        double grad0 = 0, grad1 = 0;  //ƫ����
        for (int i = 0; i < n; i++) { //�������ݼ� 
            double predicted = Predict(*theta0, *theta1, data[i].time_study);  
            grad0 += 2 * (predicted - data[i].marks);   //����ƫ���� 
            grad1 += 2 * data[i].time_study * (predicted - data[i].marks);  //����ƫ����
        } 
        printf("%d theta0: %f, theta1: %f\n", iter+1, *theta0, *theta1);       
        grad0/=n;  grad1/=n; //����ƫ����
        printf("   grad0: %f, grad1: %f\n", grad0, grad1);       
        *theta0 -= alpha * grad0;  //���²��� 
        *theta1 -= alpha * grad1;  //���²���
        if (fabs(alpha * grad0) <= esp && fabs(alpha * grad1) <= esp){ //�ж��Ƿ�ﵽ����Ҫ�� 
            printf("Converged after %d iterations.\n", iter + 1);
            break;
        }
    }  
    if (iter>=iterations)printf("Reached maximum number of iterations without converging.\n");
}
int main() {  
    DataPoint data[200];
    int n = ReadCsv("Student_Marks.csv", data);//���ļ���������
    double theta0 = 0.5, theta1 = 0.5; // ��ʼ����ֵ  
    double alpha = 0.001; // ѧϰ��  
    double esp = 0.00001; //������ֵ 
    int iterations = 100000; // ��������  
    GradientDescent(data, n, alpha, esp, iterations, &theta0, &theta1);//ѵ��ģ��  
    printf("theta0: %f, theta1: %f\n", theta0, theta1);       
    double score = Predict(theta0,theta1, 8.2);//һλѧ��ѧϰʱ����8.2����ģ�ͽ������� 
    printf("The predicted score is %lf\n", score);
    return 0;  
}
