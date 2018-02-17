
#include "fieldmap.h"


fieldMap::fieldMap()
{
  field_ = cv::Mat(600,900, CV_8UC3, cv::Scalar(255,255,255) );
}

void fieldMap::printFieldMap() {
cv::imshow("Fieldmap", field_);
}

cv::Mat fieldMap::drawFieldMap() {
//  cv::line(field_,cv::Point(0,0),cv::Point(900,0),colors::white,10);
  return field_;
}

cv::Mat fieldMap::markPosition(int x, int y, cv::Scalar color) {
   cv::circle(field_,cv::Point(x,y),4,color,-1);
   return field_;
}

cv::Mat fieldMap::markPosition(int x, int y) {
   cv::circle(field_,cv::Point(x,y),4,cv::Scalar(0,0,255),-1);
   return field_;
}
