/**
 * @file polygon.h
 * @brief Polygon类
 */

#ifndef DECOMP_POLYGON_H
#define DECOMP_POLYGON_H

#include <decomp_basis/data_type.h>

///Hyperplane类
template <int Dim>
struct Hyperplane {
  Hyperplane() {}
  Hyperplane(const Vecf<Dim>& p, const Vecf<Dim>& n) : p_(p), n_(n) {}

  /// 计算点到平面的带符号距离
  decimal_t signed_dist(const Vecf<Dim>& pt) const {
    return n_.dot(pt - p_);
  }

  /// 计算点到平面的距离
  decimal_t dist(const Vecf<Dim>& pt) const {
    return std::abs(signed_dist(pt));
  }

  /// 平面上的点
  Vecf<Dim> p_;
  /// 平面的法向量，方向性
  Vecf<Dim> n_;
};

///Hyperplane2D: 第一个是平面上的点，第二个是法向量
typedef Hyperplane<2> Hyperplane2D;
///Hyperplane3D: 第一个是平面上的点，第二个是法向量
typedef Hyperplane<3> Hyperplane3D;


///Polyhedron类
template <int Dim>
struct Polyhedron {
  ///空构造函数
  Polyhedron() {}
  ///从Hyperplane数组构造
  Polyhedron(const vec_E<Hyperplane<Dim>>& vs) : vs_(vs) {}


  ///添加Hyperplane
  void add(const Hyperplane<Dim>& v) {
    vs_.push_back(v);
  }

  /// 检查点是否在多面体内，非排他性
  bool inside(const Vecf<Dim>& pt) const {
    for (const auto& v : vs_) {
      if (v.signed_dist(pt) > epsilon_) {
        //printf("rejected pt: (%f, %f), d: %f\n",pt(0), pt(1), v.signed_dist(pt));
        return false;
      }
    }
    return true;
  }

  /// 计算多面体内的点，非排他性
  vec_Vecf<Dim> points_inside(const vec_Vecf<Dim> &O) const {
    vec_Vecf<Dim> new_O;
    for (const auto &it : O) {
      if (inside(it))
        new_O.push_back(it);
    }
    return new_O;
  }

  /// 计算法向量，用于可视化
  vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> cal_normals() const {
    vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> ns(vs_.size());
    for (size_t i = 0; i < vs_.size(); i++)
      ns[i] = std::make_pair(vs_[i].p_, vs_[i].n_); // 第一个是点，第二个是法向量
    return ns;
  }

  /// 获取Hyperplane数组
  vec_E<Hyperplane<Dim>> hyperplanes() const {
    return vs_;
  }

  /// Hyperplane数组
  vec_E<Hyperplane<Dim>> vs_; // 法向量必须朝外

};

///Polyhedron2D，由2D Hyperplane组成
typedef Polyhedron<2> Polyhedron2D;
///Polyhedron3D，由3D Hyperplane组成
typedef Polyhedron<3> Polyhedron3D;

///[A, b] 用于 \f$Ax < b\f$
template <int Dim>
struct LinearConstraint {
  ///空构造函数
  LinearConstraint() {}
  /// 直接从 \f$A, b\f$ 构造，满足 \f$Ax < b\f$
  LinearConstraint(const MatDNf<Dim>& A, const VecDf& b) : A_(A), b_(b) {}
  /**
   * @brief 从内部点和Hyperplane数组构造
   * @param p0 内部点
   * @param vs Hyperplane数组，法向量应朝外
   */
	LinearConstraint(const Vecf<Dim> p0, const vec_E<Hyperplane<Dim>>& vs) {
		const unsigned int size = vs.size();
		MatDNf<Dim> A(size, Dim);
		VecDf b(size);

		for (unsigned int i = 0; i < size; i++) {
			auto n = vs[i].n_;
			decimal_t c = vs[i].p_.dot(n);
			if (n.dot(p0) - c > 0) {
				n = -n;
				c = -c;
			}
			A.row(i) = n;
			b(i) = c;
		}

		A_ = A;
		b_ = b;
	}

  /// 使用线性约束检查点是否在多面体内
  bool inside(const Vecf<Dim> &pt) {
    VecDf d = A_ * pt - b_;
    for (unsigned int i = 0; i < d.rows(); i++) {
      if (d(i) > 0)
        return false;
    }
    return true;
  }

  /// 获取 \f$A\f$ 矩阵
  MatDNf<Dim> A() const { return A_; }

  /// 获取 \f$b\f$ 矩阵
  VecDf b() const { return b_; }

  MatDNf<Dim> A_;
  VecDf b_;
};

///LinearConstraint 2D
typedef LinearConstraint<2> LinearConstraint2D;
///LinearConstraint 3D
typedef LinearConstraint<3> LinearConstraint3D;

#endif