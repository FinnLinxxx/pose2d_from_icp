#ifndef COLORS_H
#define COLORS_H

#include "opencv/cv.h"



namespace colors
{
  extern cv::Scalar fieldGreen;
  extern cv::Scalar blue;
  extern cv::Scalar green;
  extern cv::Scalar red;
  extern cv::Scalar white;

  cv::Scalar setColor(int red, int green, int blue);

}
#endif // COLORS_H
