/**
 * @file ellipsoid.h
 * @brief 椭球类
 */

#ifndef DECOMP_ELLIPSOID_H
#define DECOMP_ELLIPSOID_H

#include <iostream>
#include <decomp_basis/data_type.h>
#include <decomp_geometry/polyhedron.h>

// 定义一个模板结构体 Ellipsoid，Dim 是维度参数
template <int Dim>
struct Ellipsoid {
  // 默认构造函数
  Ellipsoid() {}

  // 带参数的构造函数，接受一个矩阵 C 和一个向量 d
  Ellipsoid(const Matf<Dim, Dim>& C, const Vecf<Dim>& d) : C_(C), d_(d) {}

  /// 计算点到椭球中心的距离
  decimal_t dist(const Vecf<Dim>& pt) const {
    // 计算点到椭球中心的距离，使用矩阵 C 的逆乘以 (pt - d_) 并取范数
    return (C_.inverse() * (pt - d_)).norm();
  }

  /// 检查点是否在椭球内部，非排他性
  bool inside(const Vecf<Dim>& pt) const {
      // 如果点到中心的距离小于等于1，则点在椭球内部
      return dist(pt) <= 1;
  }

  /// 计算椭球内部的点，非排他性
  vec_Vecf<Dim> points_inside(const vec_Vecf<Dim> &O) const {
    vec_Vecf<Dim> new_O;
    // 遍历所有点，如果点在椭球内部，则将其加入 new_O
    for (const auto &it : O) {
      if (inside(it))
        new_O.push_back(it);
    }
    return new_O;
  }

  /// 找到最近的点
  Vecf<Dim> closest_point(const vec_Vecf<Dim> &O) const {
    Vecf<Dim> pt = Vecf<Dim>::Zero();
    decimal_t min_dist = std::numeric_limits<decimal_t>::max();
    // 遍历所有点，找到距离椭球中心最近的点
    for (const auto &it : O) {
      decimal_t d = dist(it);
      if (d < min_dist) {
        min_dist = d;
        pt = it;
      }
    }
    return pt;
  }

  /// 找到最近的点对应的最近的超平面
  Hyperplane<Dim> closest_hyperplane(const vec_Vecf<Dim> &O) const {
    const auto closest_pt = closest_point(O);
    // 计算超平面的法向量
    const auto n = C_.inverse() * C_.inverse().transpose() *
      (closest_pt - d_);
    // 返回超平面，法向量已归一化
    return Hyperplane<Dim>(closest_pt, n.normalized());
  }

  /// 沿轮廓采样 n 个点
  template<int U = Dim>
    typename std::enable_if<U == 2, vec_Vecf<U>>::type
    sample(int num) const {
    vec_Vecf<Dim> pts;
      decimal_t dyaw = M_PI*2/num;
      // 在椭球轮廓上采样点
      for(decimal_t yaw = 0; yaw < M_PI*2; yaw+=dyaw) {
        Vecf<Dim> pt;
        pt << cos(yaw), sin(yaw);
        pts.push_back(C_ * pt + d_);
    }
    return pts;
  }

  // 打印椭球的 C 矩阵和中心 d
  void print() const {
    std::cout << "C: " << C_ << std::endl;
    std::cout << "d: " << d_ << std::endl;
  }

  /// 获取椭球的体积
  decimal_t volume() const {
    // 体积等于矩阵 C 的行列式
    return C_.determinant();
  }

  /// 获取 C 矩阵
  Matf<Dim, Dim> C() const {
    return C_;
  }

  /// 获取中心 d
  Vecf<Dim> d() const {
    return d_;
  }

  // 椭球的 C 矩阵和中心 d
  Matf<Dim, Dim> C_;
  Vecf<Dim> d_;
};

// 定义二维椭球类型
typedef Ellipsoid<2> Ellipsoid2D;

// 定义三维椭球类型
typedef Ellipsoid<3> Ellipsoid3D;

#endif