/**
 * @file ellipsoid_decomp.h
 * @brief EllipsoidDecomp 类的头文件
 */
#ifndef ELLIPSOID_DECOMP_H
#define ELLIPSOID_DECOMP_H

#include <memory> // 包含智能指针等内存管理工具的头文件
#include <decomp_util/line_segment.h> // 包含线段分解工具的头文件

/**
 * @brief EllipsoidDecomp 类
 *
 * EllipsoidDecomp 类接收一个给定的路径，并使用椭球体找到围绕该路径的安全飞行走廊
 */
template <int Dim>
class EllipsoidDecomp {
public:
 /// 简单的构造函数
 EllipsoidDecomp() {}
 /**
  * @brief 基本的构造函数
  * @param origin 全局包围盒的原点
  * @param dim 全局包围盒的维度
  */
 EllipsoidDecomp(const Vecf<Dim> &origin, const Vecf<Dim> &dim) {
   global_bbox_min_ = origin; // 设置全局包围盒的最小值
   global_bbox_max_ = origin + dim; // 设置全局包围盒的最大值
 }

 /// 设置障碍物点
 void set_obs(const vec_Vecf<Dim> &obs) { obs_ = obs; }

 /// 设置包围盒的维度
 void set_local_bbox(const Vecf<Dim>& bbox) { local_bbox_ = bbox; }

 /// 获取用于膨胀的路径
 vec_Vecf<Dim> get_path() const { return path_; }

 /// 获取安全飞行走廊的多面体
 vec_E<Polyhedron<Dim>> get_polyhedrons() const { return polyhedrons_; }

 /// 获取椭球体
 vec_E<Ellipsoid<Dim>> get_ellipsoids() const { return ellipsoids_; }

 /// 获取安全飞行走廊的约束条件，形式为 \f$Ax\leq b \f$
 vec_E<LinearConstraint<Dim>> get_constraints() const {
   vec_E<LinearConstraint<Dim>> constraints;
   constraints.resize(polyhedrons_.size()); // 调整约束条件的大小
   for (unsigned int i = 0; i < polyhedrons_.size(); i++){
     const Vecf<Dim> pt = (path_[i] + path_[i+1])/2; // 计算路径中点的平均值
     constraints[i] = LinearConstraint<Dim>(pt, polyhedrons_[i].hyperplanes()); // 生成线性约束
   }
   return constraints;
 }

 /**
  * @brief 分解线程
  * @param path 要膨胀的路径
  * @param offset_x 添加到长半轴的偏移量，默认为0
  */
 void dilate(const vec_Vecf<Dim> &path, double offset_x = 0) {
   const unsigned int N = path.size() - 1; // 计算路径的段数
   lines_.resize(N); // 调整线段数组的大小
   ellipsoids_.resize(N); // 调整椭球体数组的大小
   polyhedrons_.resize(N); // 调整多面体数组的大小

   for (unsigned int i = 0; i < N; i++) {
     lines_[i] = std::make_shared<LineSegment<Dim>>(path[i], path[i+1]); // 创建线段对象
     lines_[i]->set_local_bbox(local_bbox_); // 设置局部包围盒
     lines_[i]->set_obs(obs_); // 设置障碍物点
     lines_[i]->dilate(offset_x); // 进行膨胀操作

     ellipsoids_[i] = lines_[i]->get_ellipsoid(); // 获取膨胀后的椭球体
     polyhedrons_[i] = lines_[i]->get_polyhedron(); // 获取膨胀后的多面体
   }

   path_ = path; // 保存路径

   if(global_bbox_min_.norm() != 0 || global_bbox_max_.norm() != 0) { // 如果全局包围盒不为空
     for(auto& it: polyhedrons_)
       add_global_bbox(it); // 为每个多面体添加全局包围盒
   }

 }

protected:
 template<int U = Dim>
   typename std::enable_if<U == 2>::type
   add_global_bbox(Polyhedron<Dim> &Vs) {
     //**** 沿X、Y轴添加边界

     //*** X轴
     Vs.add(Hyperplane2D(Vec2f(global_bbox_max_(0), 0), Vec2f(1, 0)));
     Vs.add(Hyperplane2D(Vec2f(global_bbox_min_(0), 0), Vec2f(-1, 0)));
     //*** Y轴
     Vs.add(Hyperplane2D(Vec2f(0, global_bbox_max_(1)), Vec2f(0, 1)));
     Vs.add(Hyperplane2D(Vec2f(0, global_bbox_min_(1)), Vec2f(0, -1)));
   }

 template<int U = Dim>
   typename std::enable_if<U == 3>::type
   add_global_bbox(Polyhedron<Dim> &Vs) {
     //**** 沿X、Y、Z轴添加边界
     //*** Z轴
     Vs.add(Hyperplane3D(Vec3f(0, 0, global_bbox_max_(2)), Vec3f(0, 0, 1)));
     Vs.add(Hyperplane3D(Vec3f(0, 0, global_bbox_min_(2)), Vec3f(0, 0, -1)));

     //*** X轴
     Vs.add(Hyperplane3D(Vec3f(global_bbox_max_(0), 0, 0), Vec3f(1, 0, 0)));
     Vs.add(Hyperplane3D(Vec3f(global_bbox_min_(0), 0, 0), Vec3f(-1, 0, 0)));
     //*** Y轴
     Vs.add(Hyperplane3D(Vec3f(0, global_bbox_max_(1), 0), Vec3f(0, 1, 0)));
     Vs.add(Hyperplane3D(Vec3f(0, global_bbox_max_(1), 0), Vec3f(0, -1, 0)));
   }

 vec_Vecf<Dim> path_; // 路径
 vec_Vecf<Dim> obs_; // 障碍物点

 vec_E<Ellipsoid<Dim>> ellipsoids_; // 椭球体
 vec_E<Polyhedron<Dim>> polyhedrons_; // 多面体
 std::vector<std::shared_ptr<LineSegment<Dim>>> lines_; // 线段

 Vecf<Dim> local_bbox_{Vecf<Dim>::Zero()}; // 局部包围盒
 Vecf<Dim> global_bbox_min_{Vecf<Dim>::Zero()}; // 全局包围盒的最小值
 Vecf<Dim> global_bbox_max_{Vecf<Dim>::Zero()}; // 全局包围盒的最大值

};

typedef EllipsoidDecomp<2> EllipsoidDecomp2D; // 2D版本的EllipsoidDecomp

typedef EllipsoidDecomp<3> EllipsoidDecomp3D; // 3D版本的EllipsoidDecomp
#endif