#include "../include/MatrixOperations.h"
#define M_PI 3.14159265358979323846
using namespace Eigen;

MatrixXd creMatrix(int rows, int cols) {
    MatrixXd m(rows, cols);
    std::cout << "请输入矩阵元素（按行输入）:\n";
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            std::cin >> m(i, j);
    return m;
}

MatrixXd creMatrixIn() {
    int r, c;
    std::cout << "输入行数: ";
    std::cin >> r;
    std::cout << "输入列数: ";
    std::cin >> c;
    return creMatrix(r, c);
}

void pMatrix(const MatrixXd& m) {
    std::cout << m << "\n";
}

MatrixXd add(const MatrixXd& a, const MatrixXd& b) {
    return a + b;
}

MatrixXd sub(const MatrixXd& a, const MatrixXd& b) {
    return a - b;
}

MatrixXd Mul(const MatrixXd& a, double k) {
    return a * k;
}

MatrixXd mul(const MatrixXd& a, const MatrixXd& b) {
    return a * b;
}

MatrixXd trans(const MatrixXd& a) {
    return a.transpose();
}

void pInfo(const MatrixXd& a) {
    std::cout << "行: " << a.rows() << "  列: " << a.cols() << "\n";
}

bool is_Empty(const MatrixXd& a) {
    return a.size() == 0;
}


double deg2rad(double degree) {return degree * (M_PI) / 180.0; }
// 3D 欧拉角变换 (ZYX 顺序)
MatrixXd eulerTransform3D(double rx, double ry, double rz,
                                 double tx, double ty, double tz,
                                 MatrixXd& point) {

    double rx_rad = deg2rad(rx);
    double ry_rad = deg2rad(ry);
    double rz_rad = deg2rad(rz);

    // 绕 Z 轴旋转 (偏航 yaw)
    Matrix3d Rz;
    Rz << std::cos(rz_rad), -std::sin(rz_rad), 0,
          std::sin(rz_rad),  std::cos(rz_rad), 0,
          0,                 0,                1;

    // 绕 Y 轴旋转 (俯仰 pitch)
    Matrix3d Ry;
    Ry <<  std::cos(ry_rad), 0, std::sin(ry_rad),
           0,                1, 0,
          -std::sin(ry_rad), 0, std::cos(ry_rad);

    // 绕 X 轴旋转 (滚转 roll)
    Matrix3d Rx;
    Rx << 1, 0,                 0,
          0, std::cos(rx_rad), -std::sin(rx_rad),
          0, std::sin(rx_rad),  std::cos(rx_rad);

    // 合成旋转矩阵 (ZYX 顺序: R = Rz * Ry * Rx)
    Matrix3d R = Rz * Ry * Rx;

    MatrixXd T(3,1);
    T(0,0) = tx;
    T(1,0) = ty;
    T(2,0) = tz;
    
    return trans(R * point + T) ;
}

