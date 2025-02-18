/**
 * @file seed_decomp.h
 * @brief SeedDecomp 类的定义文件
 */
#ifndef SEED_DECOMP_H
#define SEED_DECOMP_H

#include <decomp_util/decomp_base.h>

/**
 * @brief Seed Decomp 类
 *
 * 围绕给定点进行膨胀
 */
template <int Dim>
class SeedDecomp : public DecompBase<Dim> {
  public:
    /// 简单的构造函数
    SeedDecomp() {};
    /**
     * @brief 基本的构造函数
     * @param p 线段的一个端点
     */
    SeedDecomp(const Vecf<Dim> &p) : p_(p) {}
    /**
     * @brief 用一个球体膨胀种子
     * @param radius 机器人半径
     */
    void dilate(decimal_t radius) {
      this->ellipsoid_ = Ellipsoid<Dim>(radius * Matf<Dim, Dim>::Identity(), p_);
      this->find_polyhedron();
      add_local_bbox(this->polyhedron_);
    }

    /// 获取中心点
    Vecf<Dim> get_seed() const {
      return p_;
    }

  protected:
    /// 添加边界框
    void add_local_bbox(Polyhedron<Dim> &Vs) {
      if(this->local_bbox_.norm() == 0)
        return;

      //**** 虚拟墙壁 x-y-z
      Vecf<Dim> dir = Vecf<Dim>::UnitX();
      Vecf<Dim> dir_h = Vecf<Dim>::UnitY();

      Vecf<Dim> pp1 = p_ + dir_h * this->local_bbox_(1);
      Vecf<Dim> pp2 = p_ - dir_h * this->local_bbox_(1);
      Vs.add(Hyperplane<Dim>(pp1, dir_h));
      Vs.add(Hyperplane<Dim>(pp2, -dir_h));

      // 沿着 y 轴
      Vecf<Dim> pp3 = p_ + dir * this->local_bbox_(0);
      Vecf<Dim> pp4 = p_ - dir * this->local_bbox_(0);
      Vs.add(Hyperplane<Dim>(pp3, dir));
      Vs.add(Hyperplane<Dim>(pp4, -dir));

      // 沿着 z 轴
      if(Dim > 2) {
        Vecf<Dim> dir_v = Vecf<Dim>::UnitZ();
        Vecf<Dim> pp5 = p_ + dir_v * this->local_bbox_(2);
        Vecf<Dim> pp6 = p_ - dir_v * this->local_bbox_(2);
        Vs.add(Hyperplane<Dim>(pp5, dir_v));
        Vs.add(Hyperplane<Dim>(pp6, -dir_v));
      }
    }

    /// 种子位置
    Vecf<Dim> p_;
};

/// 2D 版本的 SeedDecomp 类
typedef SeedDecomp<2> SeedDecomp2D;

/// 3D 版本的 SeedDecomp 类
typedef SeedDecomp<3> SeedDecomp3D;

#endif