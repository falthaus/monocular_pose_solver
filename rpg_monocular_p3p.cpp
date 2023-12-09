////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////


#include <utility>
#include <limits.h>
#include "rpg_monocular_p3p.h"
#include <stdarg.h>

#include <Eigen/Dense>
using namespace Eigen;

#include "monocular_pose_estimator_lib/p3p.h"
using namespace monocular_pose_estimator;


////////////////////////////////////////////////////////////////////////////////


Eigen::Matrix3d camera_matrix;
Eigen::Matrix<double, 3, 4> world_points;


////////////////////////////////////////////////////////////////////////////////


void print_str(char* buf, int* offset, size_t len, const char* fmt, ...);

void print_matrix(char* buf, int* offset, size_t len, const MatrixXd& m);


////////////////////////////////////////////////////////////////////////////////


void set_camera_matrix(double* cm, char* buf, size_t len)
{
    // pretty sure there is a better way to do this, but I can't figure it out now

    Map<Matrix<double, 3, 3, RowMajor>> cm_mapped(cm);

    for (int c = 0; c < camera_matrix.cols(); c++)
    {
        for (int r = 0; r < camera_matrix.rows(); r++)
        {
            camera_matrix(r, c) = cm_mapped(r, c);
        }

    }

    int offset = 0;
    print_matrix(buf, &offset, len, camera_matrix);

}


////////////////////////////////////////////////////////////////////////////////


void set_world_points(double* wp, char* buf, size_t len)
{
    // pretty sure there is a better way to do this, but I can't figure it out now

    Map<Matrix<double, 3, 4, RowMajor>> wp_mapped(wp);

    for (int c = 0; c < world_points.cols(); c++)
    {
        for (int r = 0; r < world_points.rows(); r++)
        {
            world_points(r, c) = wp_mapped(r, c);
        }

    }

    int offset = 0;
    print_matrix(buf, &offset, len, world_points);

}


////////////////////////////////////////////////////////////////////////////////


void solve_p4p(float* blobs, double* R, double* t, float* reprojection_error, char* buf, size_t len)
{

    int offset = 0;
    Eigen::Matrix<double, 3, 3> feature_vectors;
    Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;
    int executed_correctly;

    Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> reprojected_points;
    Eigen::Matrix<Eigen::Matrix<double, 2, 4>, 4, 1> reprojected_blobs;
    float reprojection_errors_4pt[4];
    float reprojection_errors_1pt[4];

    Eigen::Matrix<double, 2, 4> blob_vectors;


    for (int i = 0; i < blob_vectors.cols(); i++)
    {
        blob_vectors.col(i).x() = (double)(blobs[i]);
        blob_vectors.col(i).y() = (double)(blobs[blob_vectors.cols() + i]);
    }


    for (int i = 0; i < feature_vectors.cols(); i++)
    {
        feature_vectors.col(i).x() = (blob_vectors.col(i).x() - camera_matrix(0, 2)) / camera_matrix(0, 0);
        feature_vectors.col(i).y() = (blob_vectors.col(i).y() - camera_matrix(1, 2)) / camera_matrix(1, 1);
        feature_vectors.col(i).z() = 1.0;
        feature_vectors.col(i).normalize(); // normalize in place
    }


    print_str(buf, &offset, len, "camera_matrix:\n");
    print_matrix(buf, &offset, len, camera_matrix);

    print_str(buf, &offset, len, "world_points:\n");
    print_matrix(buf, &offset, len, world_points);


    executed_correctly = P3P::computePoses(feature_vectors, world_points.leftCols(3), solutions);


    //const double ERROR_THRESHOLD = 5.0;
    float min_error_4pt = (std::numeric_limits<float>::max)();
    int best_pose_4pt = -1;
    float min_error_1pt = (std::numeric_limits<float>::max)();
    int best_pose_1pt = -1;


    for (int s = 0; s < 4; s++)         // for each solution
    {
        for (int p = 0; p < 4; p++)     // for each point
        {
            // TODO: try with just reprojecting the 4th point (unused for P3P)

            // transform from camera coordinate system to world_coordinate system:
            reprojected_points[s].col(p) = solutions[s].leftCols(3).transpose() * (world_points.col(p) - solutions[s].rightCols(1));

            // project reprojected points (homogenous coordinates):
            //reprojected_blobs[s].col(p) = (camera_matrix_K * reprojected_blobs[s].col(p)) / reprojected_blobs[s].col(p).z();
            reprojected_blobs[s].col(p).x() = camera_matrix(0, 0) * reprojected_points[s].col(p).x() / reprojected_points[s].col(p).z() + camera_matrix(0, 2);
            reprojected_blobs[s].col(p).y() = camera_matrix(1, 1) * reprojected_points[s].col(p).y() / reprojected_points[s].col(p).z() + camera_matrix(1, 2);

        }

        // calculate squared total distance (error) between received blobs and reprojected blobs
        reprojection_errors_4pt[s] = (float)((reprojected_blobs[s] - blob_vectors).squaredNorm());
        // calculate squared total error for 4th point only:
        reprojection_errors_1pt[s] = (float)((reprojected_blobs[s].col(3) - blob_vectors.col(3)).squaredNorm());

        // find pose estimate with minimum error of all points
        if (reprojection_errors_4pt[s] < min_error_4pt)
        {
            min_error_4pt = reprojection_errors_4pt[s];
            best_pose_4pt = s;
        }

        // find pose estimate with minimum error of 4th point
        if (reprojection_errors_1pt[s] < min_error_1pt)
        {
            min_error_1pt = reprojection_errors_1pt[s];
            best_pose_1pt = s;
        }


    }


    print_str(buf, &offset, len, "blob_vectors:\n");
    print_matrix(buf, &offset, len, blob_vectors);

    print_str(buf, &offset, len, "feature_vectors:\n");
    print_matrix(buf, &offset, len, feature_vectors);

    print_str(buf, &offset, len, "solutions[0]:\n");
    print_matrix(buf, &offset, len, solutions[0]);
    print_str(buf, &offset, len, "solutions[1]:\n");
    print_matrix(buf, &offset, len, solutions[1]);
    print_str(buf, &offset, len, "solutions[2]:\n");
    print_matrix(buf, &offset, len, solutions[2]);
    print_str(buf, &offset, len, "solutions[3]:\n");
    print_matrix(buf, &offset, len, solutions[3]);


    print_str(buf, &offset, len, "reprojected_blobs:\n");
    print_matrix(buf, &offset, len, reprojected_blobs[0]);
    print_matrix(buf, &offset, len, reprojected_blobs[1]);
    print_matrix(buf, &offset, len, reprojected_blobs[2]);
    print_matrix(buf, &offset, len, reprojected_blobs[3]);


    print_str(buf, &offset, len, "reprojection_errors_4pt:\n");
    for (int i = 0; i < 4; i++)
    {
        print_str(buf, &offset, len, "%d: %.3f\n", i, reprojection_errors_4pt[i]);
    }
    print_str(buf, &offset, len, "\n");

    print_str(buf, &offset, len, "reprojection_errors_1pt:\n");
    for (int i = 0; i < 4; i++)
    {
        print_str(buf, &offset, len, "%d: %.3f\n", i, reprojection_errors_1pt[i]);
    }
    print_str(buf, &offset, len, "\n");


    print_str(buf, &offset, len, "best solution (4pt): #%d\n", best_pose_4pt);
    print_str(buf, &offset, len, "best solution (1pt): #%d\n", best_pose_1pt);


    for (int i = 0; i < (3 * 3); i++)
        R[i] = solutions[best_pose_4pt].leftCols(3).data()[i];


    t[0] = solutions[best_pose_4pt](0,3);
    t[1] = solutions[best_pose_4pt](1,3);
    t[2] = solutions[best_pose_4pt](2,3);


    *reprojection_error = reprojection_errors_4pt[best_pose_4pt];

}


////////////////////////////////////////////////////////////////////////////////


// TODO: add argument to return indices of used blobs
void solve_pnp(float* blobs, unsigned int blobcount, double* R, double* t, float* reprojection_error, char* buf, size_t len)
{



}


////////////////////////////////////////////////////////////////////////////////


void print_str(char* buf, int* offset, size_t len, const char* fmt, ...)
{
    // if we use va_* macros, then we must also use the v*print* functions
    // see https://en.cppreference.com/w/cpp/utility/variadic

    va_list args;
    va_start(args, fmt);
    *offset += vsnprintf(buf + *offset, len - *offset, fmt, args);
    va_end(args);

}


////////////////////////////////////////////////////////////////////////////////


void print_matrix(char* buf, int* offset, size_t len, const MatrixXd& m)
{

    for (int r = 0; r < m.rows(); r++)
    {
        for (int c = 0; c < m.cols(); c++)
        {
            *offset += snprintf(buf + *offset, len - *offset, "%.6f    ", m(r, c));
        }
        *offset += snprintf(buf + *offset, len - *offset, "\n");
    }
    *offset += snprintf(buf + *offset, len - *offset, "\n");

}


////////////////////////////////////////////////////////////////////////////////
