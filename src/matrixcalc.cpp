#include "matrixcalc.h"
#include <cmath>
#include <Eigen/Dense>
#include <iostream>


namespace MatrixCalc{

Eigen::MatrixXd matrixTransformation(Eigen::MatrixXd toTransform, double transX, double transY, Eigen::MatrixXf Rot)
{
    //std::cout << "matrixTrans Rows " << toTransform.rows() << std::endl << "matrixTrans Cols " << toTransform.cols() << std::endl;

    //std::cout << "alphaInGrad " << alphaInGrad << std::endl;

    //std::cout << "BeofreCalc:\n" << toTransform << std::endl;

    Eigen::MatrixXd RT(3,3); RT <<  Rot(0,0),  Rot(0,1), transX,
                                    Rot(1,0),  Rot(1,1), transY,
                                    0         , 0          , 1;

    //std::cout << "RT Matrix:\n" << RT << std::endl;

    Eigen::MatrixXd erg = RT*toTransform;

    //std::cout << "erginMatrixCalc:\n" << erg << std::endl;
    return erg;
}


}
