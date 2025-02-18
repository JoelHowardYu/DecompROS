/**
 * @file decomp_base.h
 * @brief Decomp Base 类
 */
#ifndef DECOMP_BASE_H
#define DECOMP_BASE_H

#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
//#include <decomp_geometry/geometry_utils.h>

/**
 * @brief 线段类
 *
 * 这是 EllipsoidDecomp 中的基本元素
 */
template <int Dim>
class DecompBase {
  public:
    /// 空构造函数
    DecompBase() {}
    /**
     * @brief 在线段周围添加局部边界框
     * @param Dim 对应轴上的距离
     *
     * 这个虚拟边界框与线段平行，x、y、z 轴不是相对于世界坐标系，而是 x 轴平行于线段，y 轴垂直于线段和世界 z 轴，z 轴垂直于线段和 y 轴
     */
    void set_local_bbox(const Vecf<Dim>& bbox) {
      local_bbox_ = bbox;
    }

    /// 导入障碍点
    void set_obs(const vec_Vecf<Dim> &obs) {
      // 只考虑局部边界框内的点
      Polyhedron<Dim> vs;
      add_local_bbox(vs);
      obs_ = vs.points_inside(obs);
    }

    /// 获取障碍点
    vec_Vecf<Dim> get_obs() const { return obs_; }

    /// 获取椭球体
    Ellipsoid<Dim> get_ellipsoid() const { return ellipsoid_; }

    /// 获取多面体
    Polyhedron<Dim> get_polyhedron() const { return polyhedron_; }

    /**
     * @brief 膨胀线段
     * @param radius 添加到长半轴的偏移量
     */
    virtual void dilate(decimal_t radius = 0) = 0;

    /**
     * @brief 缩小多面体
     * @param shrink_distance 缩小距离
     */
    virtual void shrink(double shrink_distance) {}
 protected:
    virtual void add_local_bbox(Polyhedron<Dim> &Vs) = 0;

    void find_polyhedron() {
      //**** 找到半空间
      Polyhedron<Dim> Vs;
      vec_Vecf<Dim> obs_remain = obs_;
      while (!obs_remain.empty()) {
        const auto v = ellipsoid_.closest_hyperplane(obs_remain);
        Vs.add(v);
        vec_Vecf<Dim> obs_tmp;
        for (const auto &it : obs_remain) {
          if (v.signed_dist(it) < 0)
            obs_tmp.push_back(it);
        }
        obs_remain = obs_tmp;
        /*
           std::cout << "a: " << a.transpose() << std::endl;
           std::cout << "b: " << b << std::endl;
           */
      }

      polyhedron_ = Vs;
    }

    /// 障碍点，输入
    vec_Vecf<Dim> obs_;

    /// 输出椭球体
    Ellipsoid<Dim> ellipsoid_;
    /// 输出多面体
    Polyhedron<Dim> polyhedron_;

    /// 沿线段的局部边界框
    Vecf<Dim> local_bbox_{Vecf<Dim>::Zero()};
};
#endif