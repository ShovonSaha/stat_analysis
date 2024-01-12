#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Core>
#include <pcl_ros/point_cloud.h>
#include <Eigen/StdVector>
#include <unsupported/Eigen/NonLinearOptimization>
#include <pcl/pcl_config.h>
using namespace std;

void computeAngleDerivatives (Eigen::Matrix<float, 6, 1> &p, Eigen::Vector3f x)
{
    // Simplified matEigen::Vector3f h for near 0 angles
    float cx, cy, cz, sx, sy, sz;
    if (fabs (p (3)) < 10e-5)
    {
    //p(3) = 0;
    cx = 1.0;
    sx = 0.0;
    }
    else
    {
    cx = cos (p (3));
    sx = sin (p (3));
    }
    if (fabs (p (4)) < 10e-5)
    {
    //p(4) = 0;
    cy = 1.0;
    sy = 0.0;
    }
    else
    {
    cy = cos (p (4));
    sy = sin (p (4));
    }

    if (fabs (p (5)) < 10e-5)
    {
    //p(5) = 0;
    cz = 1.0;
    sz = 0.0;
    }
    else
    {
    cz = cos (p (5));
    sz = sin (p (5));
    }

    // Precomputed angular gradiant components. Letters correspond to Equation 6.19 [Magnusson 2009]
    Eigen::Vector3f j_ang_a_; j_ang_a_ << (-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy);
    Eigen::Vector3f j_ang_b_; j_ang_b_ << (cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy);
    Eigen::Vector3f j_ang_c_; j_ang_c_ << (-sy * cz), sy * sz, cy;
    Eigen::Vector3f j_ang_d_; j_ang_d_ << sx * cy * cz, (-sx * cy * sz), sx * sy;
    Eigen::Vector3f j_ang_e_; j_ang_e_ << (-cx * cy * cz), cx * cy * sz, (-cx * sy);
    Eigen::Vector3f j_ang_f_; j_ang_f_ << (-cy * sz), (-cy * cz), 0;
    Eigen::Vector3f j_ang_g_; j_ang_g_ << (cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0;
    Eigen::Vector3f j_ang_h_; j_ang_h_ << (sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0;

    // Precomputed angular Eigen::Vector3f hessian components. Letters correspond to Equation 6.21 and numbers correspond to row index [Magnusson 2009]
    Eigen::Vector3f h_ang_a2_; h_ang_a2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy;
    Eigen::Vector3f h_ang_a3_; h_ang_a3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), (-cx * cy);

    Eigen::Vector3f h_ang_b2_; h_ang_b2_ << (cx * cy * cz), (-cx * cy * sz), (cx * sy);
    Eigen::Vector3f h_ang_b3_; h_ang_b3_ << (sx * cy * cz), (-sx * cy * sz), (sx * sy);

    Eigen::Vector3f h_ang_c2_; h_ang_c2_ << (-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0;
    Eigen::Vector3f h_ang_c3_; h_ang_c3_ << (cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0;

    Eigen::Vector3f h_ang_d1_; h_ang_d1_ << (-cy * cz), (cy * sz), (sy);
    Eigen::Vector3f h_ang_d2_; h_ang_d2_ << (-sx * sy * cz), (sx * sy * sz), (sx * cy);
    Eigen::Vector3f h_ang_d3_; h_ang_d3_ << (cx * sy * cz), (-cx * sy * sz), (-cx * cy);

    Eigen::Vector3f h_ang_e1_; h_ang_e1_ << (sy * sz), (sy * cz), 0;
    Eigen::Vector3f h_ang_e2_; h_ang_e2_ << (-sx * cy * sz), (-sx * cy * cz), 0;
    Eigen::Vector3f h_ang_e3_; h_ang_e3_ << (cx * cy * sz), (cx * cy * cz), 0;

    Eigen::Vector3f h_ang_f1_; h_ang_f1_ << (-cy * cz), (cy * sz), 0;
    Eigen::Vector3f h_ang_f2_; h_ang_f2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0;
    Eigen::Vector3f h_ang_f3_; h_ang_f3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0;
   
    Eigen::Matrix<float, 3, 6> point_gradient;
    point_gradient.setZero ();
    point_gradient.block<3, 3>(0, 0).setIdentity ();
   
    // Calculate first derivative of Transformation Equation 6.17 w.r.t. transform vector p.
    // Derivative w.r.t. ith element of transform vector corresponds to column i, Equation 6.18 and 6.19 [Magnusson 2009]
    point_gradient (1, 3) = x.dot (j_ang_a_);
    point_gradient (2, 3) = x.dot (j_ang_b_);
    point_gradient (0, 4) = x.dot (j_ang_c_);
    point_gradient (1, 4) = x.dot (j_ang_d_);
    point_gradient (2, 4) = x.dot (j_ang_e_);
    point_gradient (0, 5) = x.dot (j_ang_f_);
    point_gradient (1, 5) = x.dot (j_ang_g_);
    point_gradient (2, 5) = x.dot (j_ang_h_);

    // Vectors from Equation 6.21 [Magnusson 2009]
    Eigen::Vector3f a, b, c, d, e, f;

    a << 0, x.dot (h_ang_a2_), x.dot (h_ang_a3_);
    b << 0, x.dot (h_ang_b2_), x.dot (h_ang_b3_);
    c << 0, x.dot (h_ang_c2_), x.dot (h_ang_c3_);
    d << x.dot (h_ang_d1_), x.dot (h_ang_d2_), x.dot (h_ang_d3_);
    e << x.dot (h_ang_e1_), x.dot (h_ang_e2_), x.dot (h_ang_e3_);
    f << x.dot (h_ang_f1_), x.dot (h_ang_f2_), x.dot (h_ang_f3_);

    Eigen::Matrix<float, 18, 6> point_hessian;
    // Calculate second derivative of Transformation Equation 6.17 w.r.t. transform vector p.
    // Derivative w.r.t. ith and jth elements of transform vector corresponds to the 3x1 block matrix starting at (3i,j), Equation 6.20 and 6.21 [Magnusson 2009]
    point_hessian.block<3, 1>(9, 3) = a.transpose();
    point_hessian.block<3, 1>(12, 3) = b.transpose();
    point_hessian.block<3, 1>(15, 3) = c.transpose();
    point_hessian.block<3, 1>(9, 4) = b.transpose();
    point_hessian.block<3, 1>(12, 4) = d.transpose();
    point_hessian.block<3, 1>(15, 4) = e.transpose();
    point_hessian.block<3, 1>(9, 5) = c.transpose();
    point_hessian.block<3, 1>(12, 5) = e.transpose();
    point_hessian.block<3, 1>(15, 5) = f.transpose();
   

}

// JE is used for calculatin the first order derivatives of the score function, based on 6.18, Magnusson 2009 book
void computeJE (const Eigen::Vector3f& x,
    const Eigen::Matrix<float, 6, 1>& p,
    Eigen::Matrix<float, 3, 6>& JE)
{
    JE.setZero();
    float X = p (3);
    float Y = p (4);
    float Z = p (5);

    Eigen::Vector3f a_angle_;
    a_angle_ << (-sin(X)*sin(Z) + cos(X)*sin(Y)*cos(Z)),
        (-sin(X)*cos(Z) - cos(X)*sin(Y)*sin(Z)), (-cos(X)*cos(Y));
    float a = x.dot(a_angle_);
   
    Eigen::Vector3f b_angle_;
    b_angle_ << (cos(X)*sin(Z) + sin(X)*sin(Y)*cos(Z)),
        (-sin(X)*sin(Y)*sin(Z) + cos(X)*cos(Z)), (-sin(X)*cos(Y));
    float b = x.dot(b_angle_);

    Eigen::Vector3f c_angle_;
    c_angle_ << (-sin(Y)*cos(Z)), (sin(Y)*sin(Z)), cos(Y);
    float c = x.dot(c_angle_);

    Eigen::Vector3f d_angle_;
    d_angle_ << (sin(X)*cos(Y)*cos(Z)), (-sin(X)*cos(Y)*sin(Z)), (sin(X)*sin(Y));
    float d = x.dot(d_angle_);

    Eigen::Vector3f e_angle_;
    e_angle_ << (-cos(X)*cos(Y)*cos(Z)), (cos(X)*cos(Y)*sin(Z)), (-cos(X)*sin(Y));
    float e = x.dot(e_angle_);

    Eigen::Vector3f f_angle_;
    f_angle_ << (-cos(Y)*sin(Z)), (-cos(Y)*cos(Z)), 0;
    float f = x.dot(f_angle_);

    Eigen::Vector3f g_angle_;
    g_angle_ << (cos(X)*cos(Z) - sin(X)*sin(Y)*sin(Z)), (-cos(X)*sin(Z) - sin(X)*sin(Y)*cos(Z)), 0;
    float g = x.dot(g_angle_);

    Eigen::Vector3f h_angle_;
    h_angle_ << (sin(X)*cos(Z) + cos(X)*sin(Y)*sin(Z)), (cos(X)*sin(Y)*cos(Z) - sin(X)*sin(Z)), 0;
    float h = x.dot(h_angle_);

    JE.setZero();
    JE(0, 0) = 1; JE(1, 1) = 1; JE(2, 2) = 1;
                    JE(1, 3) = a; JE(2, 3) = b;
    JE(0, 4) = c; JE(1, 4) = d; JE(2, 4) = e;
    JE(0, 5) = f; JE(1, 5) = g; JE(2, 5) = h;

};

// HE is used for calculating the hessian matrix, based on 6.20, Magnusson 2009 book
void computeHE (const Eigen::Vector3f& x,
    const Eigen::Matrix<float, 6, 1>& p,
    Eigen::Matrix<float, 18, 6>& HE)
{
    HE.setZero();
    float X = p (3);
    float Y = p (4);
    float Z = p (5);

    Eigen::Vector3f a;
    Eigen::Vector3f a_angle_Y_;
    a_angle_Y_ << -cos(X)*sin(Z) - sin(X)*sin(Y)*cos(Z),
        -cos(X)*cos(Z) + sin(X)*sin(Y)*sin(Z) , sin(X)*cos(Y);
    Eigen::Vector3f a_angle_Z_;
    a_angle_Z_ << -sin(X)*sin(Z) + cos(X)*sin(Y)*cos(Z),
    -cos(X)*sin(Y)*sin(Z) - sin(X)*cos(Z), -cos(X)*cos(Y);
    a << 0, x.dot(a_angle_Y_), x.dot(a_angle_Z_);

    Eigen::Vector3f b;
    Eigen::Vector3f b_angle_Y_;
    b_angle_Y_ << cos(X)*cos(Y)*cos(Z), -cos(X)*cos(Y)*sin(Z), cos(X)*sin(Y);
    Eigen::Vector3f b_angle_Z_;
    b_angle_Z_ << sin(X)*cos(Y)*cos(Z), -sin(X)*cos(Y)*sin(Z), sin(X)*sin(Y);
    b << 0, x.dot(b_angle_Y_), x.dot(b_angle_Z_);

    Eigen::Vector3f c;
    Eigen::Vector3f c_angle_Y_;
    c_angle_Y_ << (-sin(X)*cos(Z) - cos(X)*sin(Y)*sin(Z)), (sin(X)*sin(Z) - cos(X)*sin(Y)*cos(Z)), 0;
    Eigen::Vector3f c_angle_Z_;
    c_angle_Z_ << cos(X)*cos(Z) - sin(X)*sin(Y)*sin(Z), (-sin(X)*sin(Y)*cos(Z) - cos(X)*sin(Z)), 0;
    c << 0, x.dot(c_angle_Y_), x.dot(c_angle_Z_);

    Eigen::Vector3f d;
    Eigen::Vector3f d_angle_X_;
    d_angle_X_ << (-cos(Y)*cos(Z)), (cos(Y)*sin(Z)), (sin(Y));
    Eigen::Vector3f d_angle_Y_;
    d_angle_Y_ << (-sin(X)*sin(Y)*cos(Z)), (sin(X)*sin(Y)*sin(Z)), (sin(X)*cos(Y));
    Eigen::Vector3f d_angle_Z_;
    d_angle_Z_ << (cos(X)*sin(Y)*cos(Z)), (-cos(X)*sin(Y)*sin(Z)), (-cos(X)*cos(Y));
    d << x.dot(d_angle_X_), x.dot(d_angle_Y_), x.dot(d_angle_Z_);

    Eigen::Vector3f e;
    Eigen::Vector3f e_angle_X_;
    e_angle_X_ << (sin(Y)*sin(Z)), (sin(Y)*cos(Z)), 0;
    Eigen::Vector3f e_angle_Y_;
    e_angle_Y_ << (-sin(X)*cos(Y)*sin(Z)), (-sin(X)*cos(Y)*cos(Z)), 0;
    Eigen::Vector3f e_angle_Z_;
    e_angle_Z_ << (cos(X)*cos(Y)*sin(Z)), (cos(X)*cos(Y)*cos(Z)), 0;
    e << x.dot(e_angle_X_), x.dot(e_angle_Y_), x.dot(e_angle_Z_);

    Eigen::Vector3f f;
    Eigen::Vector3f f_angle_X_;
    f_angle_X_ << (-cos(Y)*cos(Z)), (cos(Y)*sin(Z)), 0;
    Eigen::Vector3f f_angle_Y_;
    f_angle_Y_ << (-cos(X)*sin(Z) - sin(X)*sin(Y)*cos(Z)), (-cos(X)*cos(Z) + sin(X)*sin(Y)*sin(Z)), 0;
    Eigen::Vector3f f_angle_Z_;
    f_angle_Z_ << (-sin(X)*sin(Z) + cos(X)*sin(Y)*cos(Z)), (-cos(X)*sin(Y)*sin(Z) - sin(X)*cos(Z)), 0;
    f << x.dot(f_angle_X_), x.dot(f_angle_Y_), x.dot(f_angle_Z_);
   
    HE.block<3, 1>(9, 3) = a;
    HE.block<3, 1>(12, 3) = b;
    HE.block<3, 1>(15, 3) = c;
    HE.block<3, 1>(9, 4) = b;
    HE.block<3, 1>(12, 4) = d;
    HE.block<3, 1>(15, 4) = e;
    HE.block<3, 1>(9, 5) = c;
    HE.block<3, 1>(12, 5) = e;
    HE.block<3, 1>(15, 5) = f;
};

struct Goblin {
    int mHealth;
    Goblin (int h) : mHealth (h) {}

    bool operator<(const Goblin& rhs) {
        return this->mHealth < rhs.mHealth;
    }
};

int main (int argc, char** argv)
{

    Eigen::Matrix3f m;
    m << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;

    Eigen::Vector3f mvec;
    mvec = m.diagonal();

    Eigen::Matrix3f mdiag;
    mdiag = mvec.asDiagonal();

    cout << mdiag << endl;

    return 0;
}