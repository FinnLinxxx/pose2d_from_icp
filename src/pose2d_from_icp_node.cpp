//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <filters/filter_chain.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include "flann/flann.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "fieldmap.h"


static Eigen::MatrixXf TR(2,2); 
static bool first_map_initialized(false);


Eigen::MatrixXd matrixTransformation(Eigen::MatrixXd toTransform, double transX, double transY, Eigen::MatrixXf& Rot)
{
    Eigen::MatrixXd RT(3,3); RT <<  Rot(0,0),  Rot(0,1), transX,
                                    Rot(1,0),  Rot(1,1), transY,
                                    0         , 0          , 1;
    return RT*toTransform;
}


struct pDists{
    int p_idx;
    int q_idx;
    double dist;
};

void removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}
 

bool sortByDists(const pDists &lhs, const pDists &rhs) { return lhs.dist < rhs.dist; }


class LaserscanToPointcloud
{
public:

  LaserscanToPointcloud()
    : filter_chain_("sensor_msgs::LaserScan")
  {
    ros::NodeHandle nh_;

    scan_sub_ = nh_.subscribe("scan", 10, &LaserscanToPointcloud::scanCallback, this);
    point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud",10,false);

    ros::NodeHandle pnh_("~");
    pnh_.param("max_range", p_max_range_, 29.0);
    pnh_.param("min_range", p_min_range_, 0.0);
    pnh_.param("drop_percentage", drop_percentage_, 0.25);
    pnh_.param("iterations", iterations_, 50);
    pnh_.param("draw_results", draw_results_, false);
    pnh_.param("seq_filter_value", seq_filter_value_, 50);

    filter_chain_.configure("scan_filter_chain", pnh_);

    pnh_.param("use_high_fidelity_projection", p_use_high_fidelity_projection_, false);

    if (p_use_high_fidelity_projection_){
      pnh_.param("target_frame", p_target_frame_, std::string("NO_TARGET_FRAME_SPECIFIED"));

      if (p_target_frame_ == "NO_TARGET_FRAME_SPECIFIED"){
        ROS_ERROR("No target frame specified! Needs to be set for high fidelity projection to work");
        p_use_high_fidelity_projection_ = false;
        return;
      }

      tfl_.reset(new tf::TransformListener());
      wait_duration_ = ros::Duration(0.5);

    }
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
   //only take x modulo pointclouds into account
   std::cout << "ptcl seq: " << scan_in->header.seq << std::endl;
   if(!(scan_in->header.seq % seq_filter_value_)) {

   std::cout << "taken" << scan_in->header.seq << std::endl;
    filter_chain_.update(*scan_in, scan_filtered_);

    cloud2_.data.clear();

    const sensor_msgs::LaserScan* scan_to_convert = &scan_filtered_;

    if (p_min_range_ > 0.0){
      scan_min_range_ = scan_filtered_;

      size_t num_scans = scan_min_range_.ranges.size();

      std::vector<float>& ranges_vec = scan_min_range_.ranges;

      float min_range = static_cast<float>(p_min_range_);

      for (size_t i = 0; i < num_scans; ++i){
        if (ranges_vec[i] < min_range){
          ranges_vec[i] = -INFINITY;
        }
      }

      scan_to_convert = &scan_min_range_;
    }

    if (p_use_high_fidelity_projection_){

      ros::Time end_time   = scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment) ;

      if(tfl_->waitForTransform(p_target_frame_, scan_in->header.frame_id, scan_in->header.stamp, wait_duration_) &&
         tfl_->waitForTransform(p_target_frame_, scan_in->header.frame_id, end_time, wait_duration_)){
        projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_to_convert, cloud2_, *tfl_, p_max_range_, laser_geometry::channel_option::Intensity);
      }else{
        ROS_WARN("Timed out waiting for transform between %s and %s for %f seconds. Unable to transform laser scan.",p_target_frame_.c_str(), scan_in->header.frame_id.c_str(), wait_duration_.toSec());
        return;
      }
    }else{
      projector_.projectLaser(*scan_to_convert, cloud2_, p_max_range_, laser_geometry::channel_option::Intensity);
    }
    if (cloud2_.data.size() > 0) {

      if(first_map_initialized) {
        point_cloud2_pub_.publish(cloud2_);
        // Convert ROS message to PCL-compatible data structure
        // ROS_INFO_STREAM("Received a cloud message with " << (void *)cloud2_->height * (void *)clodu2_->width << " points");
        const sensor_msgs::PointCloud2::Ptr input (new sensor_msgs::PointCloud2(cloud2_));
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
      
        Eigen::MatrixXf local_map_raw = cloud->getMatrixXfMap();
        removeRow(local_map_raw,3);
        removeRow(local_map_raw,2);
      
        flann::Matrix<float> dataset((float*)global_map_raw.data(), global_map_raw.cols(), 2);
        flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));  //magic number
        index.buildIndex();
  

        for(int iteration = 0; iteration < iterations_;iteration++) {
          if (iteration)
            local_map_raw = erg_map_raw;


          nRaw = global_map_raw.cols();
          mRaw = local_map_raw.cols();


          uint nn = 1; // how many nearest neightbors should be searched for.
          flann::Matrix<float> query((float*)local_map_raw.data(), local_map_raw.cols(), 2);
          flann::Matrix<int> match(new int[query.rows*nn], query.rows, nn);
          flann::Matrix<float> mindist(new float[query.rows*nn], query.rows, nn);
          index.knnSearch(query, match, mindist, nn, flann::SearchParams(128)); //another magic number, MAGIC!

          std::vector<pDists> pointsWithDists;
          for( int i = 0; i < match.rows; i++ ) {
            pointsWithDists.push_back({i,match[i][0],mindist[i][0]});
          }
          sort(pointsWithDists.begin(), pointsWithDists.end(), sortByDists);
          int quantityToDeleteFromPcl = (int) match.rows*drop_percentage_;
          pointsWithDists.erase(pointsWithDists.end()-quantityToDeleteFromPcl, pointsWithDists.end());

          Eigen::MatrixXf global_map(2,match.rows-quantityToDeleteFromPcl);
          Eigen::MatrixXf local_map(2,match.rows-quantityToDeleteFromPcl);

          int matchrows = match.rows-quantityToDeleteFromPcl;

          for(int i = 0; i < matchrows;i++) {
            global_map.col(i) << global_map_raw(0,pointsWithDists.at(i).q_idx ) , global_map_raw(1,pointsWithDists.at(i).q_idx );
          }

          for(int i = 0; i < matchrows;i++) {
            local_map.col(i) << local_map_raw(0,pointsWithDists.at(i).p_idx ) , local_map_raw(1,pointsWithDists.at(i).p_idx );
          }
          // Set Cendroit and find Transformation via SVD
          // Element addition speed up by seperation
          double q_bar[2];
          std::fill_n(q_bar, 2, 0);
          for(int i = 0; i < matchrows; i++) {
            q_bar[0] += global_map(0,i) / matchrows;
          }
          for(int i = 0; i < matchrows; i++) {
            q_bar[1] += global_map(1,i) / matchrows;
          }

          Eigen::MatrixXf q_mark(2,matchrows);
          for(int i = 0; i < matchrows; i++) {
            q_mark.col(i) << global_map(0,i) - q_bar[0], global_map(1,i) - q_bar[1];
          }

          double p_bar[2];
          std::fill_n(p_bar, 2, 0);
          for(int i = 0; i < matchrows; i++) {
            p_bar[0] += local_map(0,i) / matchrows;
          }
          for(int i = 0; i < matchrows; i++) {
            p_bar[1] += local_map(1,i) / matchrows;
          }

          Eigen::MatrixXf p_mark(2,matchrows);
          for(int i = 0; i < matchrows; i++) {
            p_mark.col(i) << local_map(0,i) - p_bar[0], local_map(1,i) - p_bar[1];
          }

          // Direct insertion for alloc reasons, Eigen::MatrixXf N = p_mark*q_mark.transpose();
          Eigen::JacobiSVD<Eigen::MatrixXf> svd(p_mark*q_mark.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);

          Rot = svd.matrixV()*svd.matrixU().transpose();
          //std::cout << "Rotation: "<< Rot << std::endl;

          Eigen::Vector2f p_barEigen(p_bar[0],
                                     p_bar[1]);
          Eigen::Vector2f q_barEigen(q_bar[0],
                                     q_bar[1]);
          trans = q_barEigen - Rot*p_barEigen ;
          //std::cout << "\nx " << trans[0] << "  |  y: " << trans[1] << "\n" << std::endl;

          Eigen::MatrixXf erg1(3,mRaw);
          for(unsigned int i = 0; i < mRaw; i++) {
            erg1.col(i) << local_map_raw(0,i), local_map_raw(1,i), 1;
          }
          Eigen::MatrixXd erg = matrixTransformation(erg1.cast <double> (),trans(0),trans(1),Rot);
 
          TR = Rot*TR;
    
          Eigen::Vector2f T(trans(0),
                            trans(1));
          TT = Rot*T + TT; 

          Eigen::MatrixXf erg_map(2,mRaw);
          for(unsigned int i = 0; i < mRaw; i++) {
            erg_map.col(i) << erg(0,i), erg(1,i);
          }

          erg_map_raw = erg_map;

          //std::cout << global_map_raw.transpose() << std::endl;
          delete[] match.ptr();  //clear
          delete[] mindist.ptr();

          if(draw_results_) { // Draw FieldMap
            fieldMap map;
            map.drawFieldMap();
            int offsetCV = 150;
            for(unsigned int i = 0; i < nRaw; i++) {
              map.markPosition((global_map_raw(0,i)*100)+offsetCV, (global_map_raw(1,i)*100)+offsetCV, cv::Scalar(255,0,0));
            }
            for(unsigned int i = 0; i < mRaw; i++) {
              map.markPosition((local_map_raw(0,i)*100)+offsetCV, (local_map_raw(1,i)*100)+offsetCV, cv::Scalar(0,0,255));
            }
            for(unsigned int i = 0; i < mRaw; i++) {
              map.markPosition((erg_map_raw(0,i)*100)+offsetCV, (erg_map_raw(1,i)*100)+offsetCV, cv::Scalar(0,255,0));
            }
            map.printFieldMap();
            cv::waitKey(0);
          }
  
        }
        global_map_raw = erg_map_raw;
        std::cout << "Rotation: "<< TR << std::endl;
        TR << 1,0,0,1;
        std::cout << "\nx " << TT[0] << "  |  y: " << TT[1] << "\n" << std::endl;
        TT[0] = 0; 
        TT[1] = 0; 

 
        } else { 
          point_cloud2_pub_.publish(cloud2_);
          const sensor_msgs::PointCloud2::Ptr input (new sensor_msgs::PointCloud2(cloud2_));
          pcl::PCLPointCloud2 pcl_pc2;
          pcl_conversions::toPCL(*input,pcl_pc2);
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
          global_map_raw = cloud->getMatrixXfMap();
          removeRow(global_map_raw,3);
          removeRow(global_map_raw,2);
          first_map_initialized = true;
        }
      }
    }
  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher point_cloud2_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::Duration wait_duration_;

  double p_max_range_;
  double p_min_range_;
  bool p_use_high_fidelity_projection_;
  bool draw_results_;
  int iterations_;
  int seq_filter_value_;
  double drop_percentage_;
  std::string p_target_frame_;

  Eigen::MatrixXf global_map_raw;
  Eigen::MatrixXf erg_map_raw;
  Eigen::MatrixXf Rot;
  Eigen::Vector2f trans;
  Eigen::Vector2f TT;
  unsigned int nRaw;
  unsigned int mRaw;

  laser_geometry::LaserProjection projector_;

  sensor_msgs::PointCloud2 cloud2_;
  sensor_msgs::LaserScan scan_min_range_;

  sensor_msgs::LaserScan scan_filtered_;
  
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
};

int main(int argc, char** argv)
{
  TR << 1,0,0,1;
  ros::init(argc, argv, "vigir_laserscan_to_pointcloud_node");

  LaserscanToPointcloud ls;

  ros::spin();
}
