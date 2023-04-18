#ifndef MAPS_HPP
#define MAPS_HPP

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud2.h>

namespace mocka {

class Maps {
public:
  typedef struct BasicInfo {
    ros::NodeHandle *nh_private;
    int sizeX;
    int sizeY;
    int sizeZ;
    int seed;
    double scale;
    sensor_msgs::PointCloud2 *output;
    pcl::PointCloud<pcl::PointXYZ> *cloud;
  } BasicInfo;

  BasicInfo getInfo() const;
  void setInfo(const BasicInfo &value);

public:
  Maps();

public:
  void generate(int type);
  //self define
private:
  BasicInfo info;

private:
  void pcl2ros();

  void perlin3D();
  void maze2D();
  void randomMapGenerate();
  void Maze3DGen();

  pcl::PointCloud<pcl::PointXYZ>::Ptr UnitRegular(double xwidth,double ywidth,double hwidth,double scale);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Unit_tilt(double xwidth,double ywidth,double hwidth,double scale,double tilt_angle,double tilt_width,double tilt_start) ;
  void IregNarrow();//
  void recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi &maze);
  void recursizeDivisionMaze(Eigen::MatrixXi &maze);
  void optimizeMap();
};

class MazePoint {
private:
  pcl::PointXYZ point;
  double dist1;
  double dist2;
  int point1;
  int point2;
  bool isdoor;

public:
  pcl::PointXYZ getPoint();
  int getPoint1();
  int getPoint2();
  double getDist1();
  double getDist2();
  void setPoint(pcl::PointXYZ p);
  void setPoint1(int p);
  void setPoint2(int p);
  void setDist1(double set);
  void setDist2(double set);
};

} // namespace mocka

#endif // MAPS_HPP
