#include "Colors.h"

namespace colors
{
  cv::Scalar fieldGreen = cv::Scalar(20,180,20);
  cv::Scalar blue = cv::Scalar(255,0,0);
  cv::Scalar green = cv::Scalar(0,255,0);
  cv::Scalar red = cv::Scalar(0,0,255);
  cv::Scalar white = cv::Scalar(255,255,255);

  cv::Scalar setColor(int red, int green, int blue) {
    return cv::Scalar(blue,red,green);
  }

}
