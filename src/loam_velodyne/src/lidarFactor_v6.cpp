#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "glog/logging.h"

struct LidarEdgeFactor
{
    LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_, double s_)
                  : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_){}

    template <typename T>
    bool operator()(const T* const q, const T* t, T* residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
        Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};

        //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        q_last_curr = q_identity.slerp(T(s), q_last_curr);
        Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T, 3, 1> de = lpa - lpb;

        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();

        //residual[0] = ((lp - lpa).cross((lp - lpb))).norm() / (lpa - lpb).norm();

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
                                       const Eigen::Vector3d last_point_b_, const double s_) 
    {
      return (new ceres::AutoDiffCostFunction<
              LidarEdgeFactor, 3, 4, 3>(
                new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_a, last_point_b;
    double s;
};


struct LidarPlaneFactor
{
    LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
                     Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
                  : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
                    last_point_m(last_point_m_), s(s_)
    {
        ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
        ljm_norm.normalize();
    }

    template <typename T>
    bool operator()(const T* const q, const T* t, T* residual) const
    {

        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
        //Eigen::Matrix<T, 3, 1> lpl{T(last_point_l.x()), T(last_point_l.y()), T(last_point_l.z())};
        //Eigen::Matrix<T, 3, 1> lpm{T(last_point_m.x()), T(last_point_m.y()), T(last_point_m.z())};
        Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

        //Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
        q_last_curr = q_identity.slerp(T(s), q_last_curr);
        Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

        Eigen::Matrix<T, 3, 1> lp;
        lp = q_last_curr * cp + t_last_curr;

        residual[0] = (lp - lpj).dot(ljm);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
                                       const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
                                       const double s_) 
    {
      return (new ceres::AutoDiffCostFunction<
              LidarPlaneFactor, 1, 4, 3>(
                new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
    }

    Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
    Eigen::Vector3d ljm_norm;
    double s;
};

class LidarFactor : public ceres::SizedCostFunction<1, 4, 3>
{
    public:
        LidarFactor() = delete;
        LidarFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_, double distort_ratio_) : curr_point(curr_point_), last_point_a(last_point_a_),
                                                                            last_point_b(last_point_b_), distort_ratio(distort_ratio_)
        {
            is_edge = true;
            computeDerivedItems();
        }

        LidarFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_, Eigen::Vector3d last_point_c_, double distort_ratio_)
                        : curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_),
                          last_point_c(last_point_c_), distort_ratio(distort_ratio_)
        {
            is_edge = false;
            computeDerivedItems();
        }

        void computeDerivedItems()
        {
            if (is_edge)
            {
                AB = last_point_b - last_point_a;
                norm_AB = AB.norm();
            }
            else
            {
                Eigen::Vector3d AB_ = last_point_b - last_point_a;
                Eigen::Vector3d AC_ = last_point_c - last_point_a;
                normalized_ABxAC = AB_.cross(AC_);
                normalized_ABxAC.normalize();
            }
        }

        Eigen::Matrix<double, 3, 4> QuaternionDerivation(const Eigen::Quaterniond &q0, const Eigen::Vector3d &point) const
        {
            const double *q = q0.coeffs().data();
            Eigen::Matrix<double, 3, 4> result;
            result(0, 0) = 2*q[1]*point[1] + 2*q[2]*point[2];
            result(0, 1) = -2*2*q[1]*point[0] + 2*q[0]*point[1] + 2*q[3]*point[2];
            result(0, 2) = -2*2*q[2]*point[0] - 2*q[3]*point[1] + 2*q[0]*point[2];
            result(0, 3) = -2*q[2]*point[1] + 2*q[1]*point[2];
            result(1, 0) = 2*q[1]*point[0] - 2*2*q[0]*point[1] - 2*q[3]*point[2];
            result(1, 1) = 2*q[0]*point[0] + 2*q[2]*point[2];
            result(1, 2) = 2*q[3]*point[0] - 2*2*q[2]*point[1] + 2*q[1]*point[2];
            result(1, 3) = 2*q[2]*point[0] - 2*q[0]*point[2];
            result(2, 0) = 2*q[2]*point[0] + 2*q[3]*point[1] - 2*2*q[0]*point[2];
            result(2, 1) = -2*q[3]*point[0] + 2*q[2]*point[1] - 2*2*q[1]*point[2];
            result(2, 2) = 2*q[0]*point[0] + 2*q[1]*point[1];
            result(2, 3) = -2*q[1]*point[0] + 2*q[0]*point[1];

            return result;
        }

        Eigen::Matrix3d skewSymmetic(const Eigen::Vector3d &v) const
        {
            Eigen::Matrix3d result;
            result << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
            return result;
        }

        void QuaternionSlerpWithIdentity(const Eigen::Quaterniond &q, const double s, Eigen::Quaterniond &result_q, double *jacobian) const
        {
            const double d = Eigen::Quaterniond::Identity().dot(q);
            const double absD = std::abs(d);
            const double one = 1 - 1e-15;

            double scale0, scale1, theta, sinTheta;

            if (absD >= one)
            {
                scale0 = 1.0 - s;
                scale1 = s;
            }
            else
            {
                theta = std::acos(absD);
                sinTheta = std::sin(theta);
                scale0 = std::sin((1.0 - s) * theta) / sinTheta;
                scale1 = std::sin(s * theta) / sinTheta;
            }
            
            if (d < 0)
                scale1 = -scale1;
            
            result_q.x() = scale1 * q.x();
            result_q.y() = scale1 * q.y();
            result_q.z() = scale1 * q.z();
            result_q.w() = scale1 * q.w() + scale0;

            if (jacobian)
            {
                Eigen::Map<Eigen::Matrix4d> jaco(jacobian);
                double scale0_2_q3 = 0, scale1_2_q3 = 0;
                if (absD >= one)
                {
                    double tmp_theta = std::acos(one);
                    scale0_2_q3 = ((1-s)*std::cos((1-s)*tmp_theta)*std::sin(tmp_theta) - std::sin((1-s)*tmp_theta)*std::cos(tmp_theta)) / 
                                  (std::sin(tmp_theta)*std::sin(tmp_theta)) * -1.0 / std::sqrt(1 - one*one);
                    scale1_2_q3 = (s*std::cos(s*tmp_theta)*std::sin(tmp_theta) - std::sin(s*tmp_theta)*std::cos(tmp_theta)) /
                                  (std::sin(tmp_theta)*std::sin(tmp_theta)) * -1.0 / std::sqrt(1 - one*one);
                }
                else
                {
                    scale0_2_q3 = ((1-s)*std::cos((1-s)*theta)*std::sin(theta) - std::sin((1-s)*theta)*std::cos(theta)) / 
                                  (std::sin(theta)*std::sin(theta)) * -1.0 / std::sqrt(1 - absD*absD);
                    scale1_2_q3 = (s*std::cos(s*theta)*std::sin(theta) - std::sin(s*theta)*std::cos(theta)) /
                                  (std::sin(theta)*std::sin(theta)) * -1.0 / std::sqrt(1 - absD*absD);
                }

                if (d < 0)
                {
                    scale0_2_q3 *= -1;
                    scale1_2_q3 *= -1;
                }
                jaco = scale1 * Eigen::Matrix4d::Identity();
                jaco(0, 3) = scale1_2_q3 * q.x();
                jaco(1, 3) = scale1_2_q3 * q.y();
                jaco(2, 3) = scale1_2_q3 * q.z();
                jaco(3, 3) += scale1_2_q3 * q.w() + scale0_2_q3;
            }
        }

        bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            Eigen::Map<const Eigen::Quaterniond> q_curr_last(parameters[0]);
            Eigen::Map<const Eigen::Vector3d>    t_curr_last(parameters[1]);

            double *jaco_slerp_array = new double[16];
            Eigen::Map<Eigen::Matrix4d> jaco_slerp(jaco_slerp_array);
            Eigen::Quaterniond q_point_last;

            if (jacobians)
                QuaternionSlerpWithIdentity(q_curr_last, distort_ratio, q_point_last, jaco_slerp_array);
            else
                QuaternionSlerpWithIdentity(q_curr_last, distort_ratio, q_point_last, NULL);

            Eigen::Vector3d t_point_last = distort_ratio * t_curr_last;
            Eigen::Vector3d point_start = q_point_last.inverse() * (curr_point - t_point_last);

            if (is_edge)
            {
                // edge feature
                Eigen::Vector3d OA = last_point_a - point_start;
                Eigen::Vector3d OB = last_point_b - point_start;
                Eigen::Vector3d OAxOB = OA.cross(OB);
                double norm_OAxOB = OAxOB.norm();
                residuals[0] = norm_OAxOB / norm_AB;

                if (jacobians)
                {
                    Eigen::Matrix3d R_last_curr = q_curr_last.toRotationMatrix().transpose();
                    Eigen::Matrix3d R_point_last = q_point_last.toRotationMatrix();
                    Eigen::Matrix<double, 4, 4> tmp_m = -Eigen::Matrix<double, 4, 4>::Identity();
                    tmp_m(3, 3) = 1;
                    Eigen::Matrix3d jaco_t2t = distort_ratio * Eigen::Matrix3d::Identity();
                    Eigen::Matrix<double, 3, 4> jaco_t2q = Eigen::Matrix<double, 3, 4>::Zero();
                    Eigen::Matrix<double, 3, 4> jaco_p2q = QuaternionDerivation(q_point_last.conjugate(), curr_point-t_point_last) * tmp_m * jaco_slerp
                                                         - R_point_last.transpose() * jaco_t2q;
                    Eigen::Matrix3d jaco_p2t = -R_point_last.transpose() * jaco_t2t;
                    Eigen::Matrix<double, 3, 4> jaco_OAxOB2q = -skewSymmetic(last_point_a) * jaco_p2q + skewSymmetic(last_point_b) * jaco_p2q;
                    Eigen::Matrix3d jaco_OAxOB2t = -skewSymmetic(last_point_a) * jaco_p2t + skewSymmetic(last_point_b) * jaco_p2t;
                    double norm_OAxOB = OAxOB.norm();
                    Eigen::RowVector3d jaco_r2OAxOB = OAxOB.transpose() / norm_OAxOB / norm_AB;
                    if (jacobians[0])
                    {
                        Eigen::Map<Eigen::RowVector4d> jaco_q(jacobians[0]);
                        jaco_q = jaco_r2OAxOB * jaco_OAxOB2q;
                    }
                    if (jacobians[1])
                    {
                        Eigen::Map<Eigen::RowVector3d> jaco_t(jacobians[1]);
                        jaco_t = jaco_r2OAxOB * jaco_OAxOB2t;
                    }
                }
            }
            else
            {
                // plane feature
                Eigen::Vector3d OA = last_point_a - point_start;
                double normalized_ABxAC_dot_OA = normalized_ABxAC.dot(OA);
                residuals[0] = std::abs(normalized_ABxAC_dot_OA);

                if (jacobians)
                {
                    // TODO: repeat code as above, should be optimized
                    Eigen::Matrix3d R_last_curr = q_curr_last.toRotationMatrix().transpose();
                    Eigen::Matrix3d R_point_last = q_point_last.toRotationMatrix();
                    Eigen::Matrix<double, 4, 4> tmp_m = -Eigen::Matrix<double, 4, 4>::Identity();
                    tmp_m(3, 3) = 1;
                     Eigen::Matrix3d jaco_t2t = distort_ratio * Eigen::Matrix3d::Identity();
                    Eigen::Matrix<double, 3, 4> jaco_t2q = Eigen::Matrix<double, 3, 4>::Zero();
                    Eigen::Matrix<double, 3, 4> jaco_p2q = QuaternionDerivation(q_point_last.conjugate(), curr_point-t_point_last) * tmp_m * jaco_slerp
                                                         - R_point_last.transpose() * jaco_t2q;
                    Eigen::Matrix3d jaco_p2t = -R_point_last.transpose() * jaco_t2t;

                    if (jacobians[0])
                    {
                        Eigen::Map<Eigen::RowVector4d> jaco_r2q(jacobians[0]);
                        jaco_r2q = -normalized_ABxAC.transpose() * jaco_p2q;
                        if (normalized_ABxAC_dot_OA < 0)
                            jaco_r2q *= -1;
                    }
                    if (jacobians[1])
                    {
                        Eigen::Map<Eigen::RowVector3d> jaco_r2t(jacobians[1]);
                        jaco_r2t = -normalized_ABxAC.transpose() * jaco_p2t;
                        if (normalized_ABxAC_dot_OA < 0)
                            jaco_r2t *= -1;
                    }
                }
            }

            return true;
        }

        Eigen::Vector3d curr_point;
        double distort_ratio;
        bool   is_edge;

        // derived items
        Eigen::Vector3d last_point_a;
        Eigen::Vector3d last_point_b;
        Eigen::Vector3d last_point_c;
        Eigen::Vector3d AB;
        double norm_AB;
        Eigen::Vector3d normalized_ABxAC;
};