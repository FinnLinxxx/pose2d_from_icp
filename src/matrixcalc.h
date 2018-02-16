#ifndef MATRIXCALC_H
#define MATRIXCALC_H

#include <vector>
#include <Eigen/Dense>


namespace MatrixCalc
{
    Eigen::MatrixXd matrixTransformation(Eigen::MatrixXd toTransform, double transX, double transY, Eigen::MatrixXf Rot);

}

#endif // MATRIXCALC_H
