#include <iostream>
#include "../include/MatrixOperations.h"
#include <stdlib.h>
#include <unistd.h>
int main() {
    std::cout << "=== 矩阵计算器 ===\n";

    std::cout << "初始化矩阵A\n";
    auto A = creMatrixIn();
    std::cout << "初始化矩阵B\n";
    auto B = creMatrixIn();
    MatrixXd ans;
		bool flag = 1;
    int choice, src1, src2, dest;
    double c;

    do {
        std::system("clear");
        if(flag == 0){
        		std::cout << "数学错误！！！\n";
        		flag = 1;
        }
        std::cout << "\n--- 运算 ---\n";
        std::cout << "1. 加法\n";
        std::cout << "2. 减法\n";
        std::cout << "3. 转置\n";
        std::cout << "4. 数乘\n";
        std::cout << "5. 矩阵乘法\n";
        std::cout << "6. 3D 欧拉变换\n";
        std::cout << "\n--- 管理 ---\n";
        std::cout << "7. 查看矩阵\n";
        std::cout << "8. 矩阵信息\n";
        std::cout << "9. 重新输入矩阵\n";
        std::cout << "0. 退出\n";
        std::cout << "\n选择操作: ";
        std::cin >> choice;

        switch (choice) {
            case 1:  // 加法
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): 第一个矩阵? ";
                std::cin >> src1;
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): 第二个矩阵? ";
                std::cin >> src2;
                
                {
                    MatrixXd* m1 = (src1==1) ? &A : (src1==2) ? &B : &ans;
                    MatrixXd* m2 = (src2==1) ? &A : (src2==2) ? &B : &ans;
                    
                    if (m1->rows() == m2->rows() && m1->cols() == m2->cols()) {
                        ans = add(*m1, *m2);
                        std::cout << "\n结果 =\n";
                        pMatrix(ans);
                        
                        std::cout << "\n保存到? (1=A, 2=B, 3=仅ans, 0=不保存): ";
                        std::cin >> dest;
                        if (dest == 1) A = ans;
                        else if (dest == 2) B = ans;
                    } else {
                    		flag = 0;
                    }
                }
                break;

            case 2:  // 减法
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): 被减数? ";
                std::cin >> src1;
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): 减数? ";
                std::cin >> src2;
                
                {
                    MatrixXd* m1 = (src1==1) ? &A : (src1==2) ? &B : &ans;
                    MatrixXd* m2 = (src2==1) ? &A : (src2==2) ? &B : &ans;
                    
                    if (m1->rows() == m2->rows() && m1->cols() == m2->cols()) {
                        ans = sub(*m1, *m2);
                        std::cout << "\n结果 =\n";
                        pMatrix(ans);
                        
                        std::cout << "\n保存到? (1=A, 2=B, 3=仅ans, 0=不保存): ";
                        std::cin >> dest;
                        if (dest == 1) A = ans;
                        else if (dest == 2) B = ans;
                    } else {
                        flag = 0;
                    }
                }
                break;

            case 3:  // 转置
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): ";
                std::cin >> src1;
                
                {
                    MatrixXd* m = (src1==1) ? &A : (src1==2) ? &B : &ans;
                    ans = trans(*m);
                    std::cout << "\n转置结果 =\n";
                    pMatrix(ans);
                    
                    std::cout << "\n保存到? (1=A, 2=B, 3=仅ans, 0=不保存): ";
                    std::cin >> dest;
                    if (dest == 1) A = ans;
                    else if (dest == 2) B = ans;
                }
                break;

            case 4:  // 数乘
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): ";
                std::cin >> src1;
                std::cout << "输入标量: ";
                std::cin >> c;
                
                {
                    MatrixXd* m = (src1==1) ? &A : (src1==2) ? &B : &ans;
                    ans = Mul(*m, c);
                    std::cout << "\n结果 =\n";
                    pMatrix(ans);
                    
                    std::cout << "\n保存到? (1=A, 2=B, 3=仅ans, 0=不保存): ";
                    std::cin >> dest;
                    if (dest == 1) A = ans;
                    else if (dest == 2) B = ans;
                }
                break;

            case 5:  // 矩阵乘法
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): 左矩阵? ";
                std::cin >> src1;
                std::cout << "选择矩阵 (1=A, 2=B, 3=ans): 右矩阵? ";
                std::cin >> src2;
                
                {
                    MatrixXd* m1 = (src1==1) ? &A : (src1==2) ? &B : &ans;
                    MatrixXd* m2 = (src2==1) ? &A : (src2==2) ? &B : &ans;
                    
                    if (m1->cols() == m2->rows()) {
                        ans = mul(*m1, *m2);
                        std::cout << "\n结果 =\n";
                        pMatrix(ans);
                        
                        std::cout << "\n保存到? (1=A, 2=B, 3=仅ans, 0=不保存): ";
                        std::cin >> dest;
                        if (dest == 1) A = ans;
                        else if (dest == 2) B = ans;
                    } else {
                        flag = 0;
                    }
                }
                break;

            case 6:  // 3D欧拉变换
            {
                double rx, ry, rz, tx, ty, tz, x, y, z;
                std::cout << "输入绕 X 轴旋转角度 (roll): "; std::cin >> rx;
                std::cout << "输入绕 Y 轴旋转角度 (pitch): "; std::cin >> ry;
                std::cout << "输入绕 Z 轴旋转角度 (yaw): "; std::cin >> rz;
                std::cout << "输入平移 X: "; std::cin >> tx;
                std::cout << "输入平移 Y: "; std::cin >> ty;
                std::cout << "输入平移 Z: "; std::cin >> tz;
                std::cout << "输入点 X: "; std::cin >> x;
                std::cout << "输入点 Y: "; std::cin >> y;
                std::cout << "输入点 Z: "; std::cin >> z;

                MatrixXd point(3,1);
                point << x, y, z;
                ans = eulerTransform3D(rx, ry, rz, tx, ty, tz, point);

                std::cout << "\n变换后点坐标:\n";
                pMatrix(ans);
                
                std::cout << "\n保存到? (1=A, 2=B, 3=仅ans, 0=不保存): ";
                std::cin >> dest;
                if (dest == 1) A = ans;
                else if (dest == 2) B = ans;
                break;
            }

            case 7:  // 查看矩阵
                std::cout << "\n选择查看 (1=A, 2=B, 3=ans, 4=全部): ";
                std::cin >> src1;
                if (src1 == 1 || src1 == 4) {
                    std::cout << "\n矩阵 A =\n";
                    pMatrix(A);
                }
                if (src1 == 2 || src1 == 4) {
                    std::cout << "\n矩阵 B =\n";
                    pMatrix(B);
                }
                if (src1 == 3 || src1 == 4) {
                    if (!is_Empty(ans)) {
                        std::cout << "\n矩阵 ans =\n";
                        pMatrix(ans);
                    } else {
                        std::cout << "\nans 为空\n";
                    }
                }
                break;

            case 8:  // 矩阵信息
                std::cout << "\n--- 矩阵 A ---\n";
                pInfo(A);
                std::cout << (is_Empty(A) ? "Empty\n" : "Not Empty\n");
                
                std::cout << "\n--- 矩阵 B ---\n";
                pInfo(B);
                std::cout << (is_Empty(B) ? "Empty\n" : "Not Empty\n");
                
                if (!is_Empty(ans)) {
                    std::cout << "\n--- 矩阵 ans ---\n";
                    pInfo(ans);
                    std::cout << "Not Empty\n";
                }
                break;

            case 9:  // 重新输入
                std::cout << "重新输入哪个矩阵? (1=A, 2=B): ";
                std::cin >> src1;
                if (src1 == 1) {
                    std::cout << "输入新的矩阵A\n";
                    A = creMatrixIn();
                } else if (src1 == 2) {
                    std::cout << "输入新的矩阵B\n";
                    B = creMatrixIn();
                }
                break;

            case 0:
                std::cout << "再见!\n";
                break;

            default:
                std::cout << "无效选择\n";
        }
    } while (choice != 0);

    return 0;
}
