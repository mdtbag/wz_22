#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/FFT>
#include <iostream>
using namespace Eigen;


MatrixXd creMatrix(int rows, int cols);
MatrixXd creMatrixIn();
void pMatrix(const MatrixXd& m);

MatrixXd add(const MatrixXd& a, const MatrixXd& b);
MatrixXd sub(const MatrixXd& a, const MatrixXd& b);
MatrixXd Mul(const MatrixXd& a, double k);
MatrixXd mul(const MatrixXd& a, const MatrixXd& b);
MatrixXd trans(const MatrixXd& a);

void pInfo(const MatrixXd& a);
bool is_Empty(const MatrixXd& a);

// 3D 欧拉角变换 (rx, ry, rz in degrees, tx, ty, tz in units)
MatrixXd eulerTransform3D(double rx, double ry, double rz, 
                                 double tx, double ty, double tz, 
                                 MatrixXd& point);
#endif
