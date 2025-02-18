#ifndef DECOMP_ROS_UTILS_H
#define DECOMP_ROS_UTILS_H

// 包含必要的头文件
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace DecompROS {

// 将向量转换为路径消息
template <int Dim> 
nav_msgs::msg::Path vec_to_path(const vec_Vecf<Dim> &vs) {
  nav_msgs::msg::Path path; // 创建一个路径消息对象
  for (const auto& it : vs) { // 遍历输入的向量
    geometry_msgs::msg::PoseStamped pose; // 创建一个姿态消息对象
    pose.pose.position.x = it(0); // 设置姿态的位置x坐标
    pose.pose.position.y = it(1); // 设置姿态的位置y坐标
    pose.pose.position.z = Dim == 2 ? 0 : it(2); // 如果维度是2，z坐标设为0，否则设为it(2)
    pose.pose.orientation.w = 1.0; // 设置姿态的朝向w分量
    pose.pose.orientation.x = 0.0; // 设置姿态的朝向x分量
    pose.pose.orientation.y = 0.0; // 设置姿态的朝向y分量
    pose.pose.orientation.z = 0.0; // 设置姿态的朝向z分量

    path.poses.push_back(pose); // 将姿态消息添加到路径消息中
  }

  return path; // 返回路径消息
}

// 将向量转换为点云消息

inline sensor_msgs::msg::PointCloud2 vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(pts.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto& pt : pts) {
    *iter_x = pt(0);
    *iter_y = pt(1);
    *iter_z = pt(2);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}


// 将点云消息转换为向量
inline vec_Vec3f cloud_to_vec(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud->height * cloud->width);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");

  for (size_t i = 0; i < pts.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    pts[i](0) = *iter_x;
    pts[i](1) = *iter_y;
    pts[i](2) = *iter_z;
  }

  return pts;
}

// 将ROS消息转换为多面体
inline Polyhedron3D ros_to_polyhedron(const decomp_ros_msgs::msg::Polyhedron& msg){
  Polyhedron3D poly; // 创建一个多面体对象
  for(unsigned int i = 0; i < msg.points.size(); i++){ // 遍历消息中的点
    Vec3f pt(msg.points[i].x, msg.points[i].y, msg.points[i].z); // 创建一个点
    Vec3f n(msg.normals[i].x, msg.normals[i].y, msg.normals[i].z); // 创建一个法向量
    poly.add(Hyperplane3D(pt, n)); // 将点和法向量添加到多面体中
  }
  return poly; // 返回多面体
}

// 将ROS消息数组转换为多面体数组
inline vec_E<Polyhedron3D> ros_to_polyhedron_array(const decomp_ros_msgs::msg::PolyhedronArray& msg) {
  vec_E<Polyhedron3D> polys(msg.polyhedrons.size()); // 创建一个多面体数组对象

  for(size_t i = 0; i < msg.polyhedrons.size(); i++) // 遍历消息中的多面体
    polys[i] = ros_to_polyhedron(msg.polyhedrons[i]); // 将每个多面体消息转换为多面体对象

  return polys; // 返回多面体数组
}

// 将2D多面体转换为ROS消息
inline decomp_ros_msgs::msg::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
  decomp_ros_msgs::msg::Polyhedron msg; // 创建一个多面体消息对象
  for (const auto &p : poly.hyperplanes()) { // 遍历多面体中的超平面
    geometry_msgs::msg::Point pt, n; // 创建点和法向量消息对象
    pt.x = p.p_(0); // 设置点的x坐标
    pt.y = p.p_(1); // 设置点的y坐标
    pt.z = 0; // 设置点的z坐标为0
    n.x = p.n_(0); // 设置法向量的x分量
    n.y = p.n_(1); // 设置法向量的y分量
    n.z = 0; // 设置法向量的z分量为0
    msg.points.push_back(pt); // 将点添加到消息中
    msg.normals.push_back(n); // 将法向量添加到消息中
  }

  geometry_msgs::msg::Point pt1, n1; // 创建额外的点和法向量消息对象
  pt1.x = 0, pt1.y = 0, pt1.z = 0.01; // 设置点的坐标
  n1.x = 0, n1.y = 0, n1.z = 1; // 设置法向量的分量
  msg.points.push_back(pt1); // 将点添加到消息中
  msg.normals.push_back(n1); // 将法向量添加到消息中

  geometry_msgs::msg::Point pt2, n2; // 创建额外的点和法向量消息对象
  pt2.x = 0, pt2.y = 0, pt2.z = -0.01; // 设置点的坐标
  n2.x = 0, n2.y = 0, n2.z = -1; // 设置法向量的分量
  msg.points.push_back(pt2); // 将点添加到消息中
  msg.normals.push_back(n2); // 将法向量添加到消息中

  return msg; // 返回多面体消息
}

// 将3D多面体转换为ROS消息
inline decomp_ros_msgs::msg::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly){
  decomp_ros_msgs::msg::Polyhedron msg; // 创建一个多面体消息对象
  for (const auto &p : poly.hyperplanes()) { // 遍历多面体中的超平面
    geometry_msgs::msg::Point pt, n; // 创建点和法向量消息对象
    pt.x = p.p_(0); // 设置点的x坐标
    pt.y = p.p_(1); // 设置点的y坐标
    pt.z = p.p_(2); // 设置点的z坐标
    n.x = p.n_(0); // 设置法向量的x分量
    n.y = p.n_(1); // 设置法向量的y分量
    n.z = p.n_(2); // 设置法向量的z分量
    msg.points.push_back(pt); // 将点添加到消息中
    msg.normals.push_back(n); // 将法向量添加到消息中
  }

  return msg; // 返回多面体消息
}

// 将多面体数组转换为ROS消息数组
template <int Dim>
decomp_ros_msgs::msg::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
  decomp_ros_msgs::msg::PolyhedronArray msg; // 创建一个多面体消息数组对象
  for (const auto &v : vs) // 遍历多面体数组
    msg.polyhedrons.push_back(polyhedron_to_ros(v)); // 将每个多面体转换为消息并添加到数组中
  return msg; // 返回多面体消息数组
}

// 将椭球体数组转换为ROS消息数组
template <int Dim>
decomp_ros_msgs::msg::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>>& Es) {
  decomp_ros_msgs::msg::EllipsoidArray ellipsoids;
  for (unsigned int i = 0; i < Es.size(); i++) {
    decomp_ros_msgs::msg::Ellipsoid ellipsoid;
    auto d = Es[i].d();
    ellipsoid.d[0] = d(0);
    ellipsoid.d[1] = d(1);
    ellipsoid.d[2] = Dim == 2 ? 0 : d(2);

    auto C = Es[i].C();
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        if (x < Dim && y < Dim)
          ellipsoid.e[3 * x + y] = C(x, y);
        else
          ellipsoid.e[3 * x + y] = 0;
      }
    }
    ellipsoids.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}
}

#endif
