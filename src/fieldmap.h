#ifndef FIELDMAP_H
#define FIELDMAP_H


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


class fieldMap
{
public:
  fieldMap();
  cv::Mat drawFieldMap();
  void printFieldMap();
  cv::Mat markPosition(int x, int y);
  cv::Mat markPosition(int x, int y, cv::Scalar color);

private:
  cv::Mat field_;
};

#endif // FIELDMAP_H
