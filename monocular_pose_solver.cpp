/*
 * monocular_pose_solver.cpp
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */



#include "monocular_pose_solver.h"

#include <utility>
#include <limits.h>
#include <stdarg.h>

#include <Eigen/Dense>
using namespace Eigen;

#include "monocular_pose_estimator_lib/p3p.h"
#include "monocular_pose_estimator_lib/pose_estimator.h"
using namespace monocular_pose_estimator;



Matrix3d camera_matrix;
Matrix<double, 3, 4> world_points;

PoseEstimator pose_estimator;



/*
 * Helper functions for "pretty printing" into string buffer
 *
 */
void print_str(char* buf, int* offset, size_t len, const char* fmt, ...);
void print_matrix(char* buf, int* offset, size_t len, const MatrixXd& m, const unsigned precision = 6);




void set_camera_matrix(double cm[3*3], char* buf, size_t len)
{
    int offset = 0;

    for (int r = 0; r < camera_matrix.rows(); r++)
    {
        for (int c = 0; c < camera_matrix.cols(); c++)
        {
            camera_matrix(r,c) = cm[r*camera_matrix.cols()+c];
            pose_estimator.camera_matrix_K_(r,c) = camera_matrix(r,c);
        }
    }

    print_str(buf, &offset, len, "camera_matrix:\n");
    print_matrix(buf, &offset, len, camera_matrix);
    print_str(buf, &offset, len, "\n");

}




void set_world_points(double wp[3*4], char* buf, size_t len)
{
    int offset = 0;

    for (int r = 0; r < world_points.rows(); r++)
    {
        for (int c = 0; c < world_points.cols(); c++)
        {
            world_points(r, c) = wp[r*world_points.cols()+c];
        }
    }

    // Marker points:
    Matrix<Vector4d, 4, 1> markers;
    //
    for (int i=0; i<markers.rows(); i++)
    {
        markers[i].head(3) = world_points.col(i);
        markers[i](3) = 1.0;
    }
    //
    pose_estimator.setMarkerPositions(markers);

    print_str(buf, &offset, len, "world_points:\n");
    print_matrix(buf, &offset, len, world_points);
    print_str(buf, &offset, len, "\n");


    print_str(buf, &offset, len, "markers:\n");
    for (int i=0; i<markers.rows(); i++)
    {
        print_matrix(buf, &offset, len, markers[i].transpose());
    }
    print_str(buf, &offset, len, "\n");

}





/*
computePoses() provides matrices for transforming points from the camera to the world frame
i.e.: camera frame -> world frame
*/

//int solve_p4p(float blobs[2][4], double* Rmat, double* tvec, float* reprojection_error, char* buf, size_t len)
int solve_p4p(float blobs[2*4], double Rmat[3*3], double tvec[3*1], float* reprojection_error, char* buf, size_t len)
{

    int offset = 0;
    Matrix<double, 3, 3> feature_vectors;
    Matrix<Matrix<double, 3, 4>, 4, 1> solutions;
    int executed_correctly;

    Matrix<Matrix<double, 3, 4>, 4, 1> reprojected_points;
    Matrix<Matrix<double, 2, 4>, 4, 1> reprojected_blobs;
    float reprojection_errors_4pt[4];
    float reprojection_errors_1pt[4];

    Matrix<double, 2, 4> blob_vectors;


    // 'blobs' is row-major aka 'C' (numpy default)
    // ('row-major': rows are contiguous in memory)
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


    // copy Rmat to output array
    for(int r=0; r<3; r++)
    {
        for(int c=0; c<3; c++)
        {
            // Note: Eigen uses colum-major storage order
            // ('column-major': columns are contiguous in memory)

            // copy one-to-one:
            //R[c*3+r] = solutions[best_pose_4pt](r,c);

            // copy transposed
            Rmat[r*3+c] = solutions[best_pose_4pt](r,c);
        }
    }

    // copy tvec to output array
    for(int i=0; i<3; i++)
    {
        tvec[i] = solutions[best_pose_4pt](i,3);
    }

    // copy minimum achieved reprojection error
    *reprojection_error = reprojection_errors_4pt[best_pose_4pt];


    // Return value:
    // 0: success
    return 0;
}



/*
void solve_pnp(float* blobs, unsigned int blobcount, double* R, double* t, float* reprojection_error, char* buf, size_t len)
{

    // TODO: add argument to return indices of used blobs

    // TODO: implement
    ;

}
*/



void optimize_pose(float* blobs, unsigned blobcount, unsigned* correspondences, double* Rmat, double* tvec, char* buf, size_t len, unsigned max_iter)
{

    int offset = 0;

    Matrix4d pose;


    // Correspondences:
    // Corresponces between the LEDs/marker positions (first column)
    // and the image detections (second column) using one-based counting
    pose_estimator.setCorrespondences(Map<Matrix<uint32_t, 4, 2, RowMajor>>(correspondences));
    //
    print_str(buf, &offset, len, "correspondences:\n");
    print_matrix(buf, &offset, len, Map<Matrix<uint32_t, 4, 2, RowMajor>>(correspondences).cast <double> (), 0);



    // Pose:
    //                      | r11 r12 r13 | tx |
    //      | R | t |       | r21 r22 r23 | ty |
    //      | 0 | 1 |       | r31 r32 r33 | tz |
    //                      |  0   0   0  |  1 |
    //
    Map<Matrix<double, 3, 3, RowMajor>> Rmat_mapped(Rmat);
    Map<Matrix<double, 3, 1>> tvec_mapped(tvec);
    //
    pose.block(0,0,3,3) = Rmat_mapped;
    pose.block(0,3,3,1) = tvec_mapped;
    pose.row(3) << 0.0, 0.0, 0.0, 1.0;
    //
    pose_estimator.setPredictedPose(pose);



    // Image points:
    Matrix<Vector2d, Dynamic, 1> image_points;
    image_points.resize(blobcount);
    //
    for (int i = 0; i < image_points.rows(); i++)
    {
       image_points(i).x() = (double)(blobs[i]);
       image_points(i).y() = (double)(blobs[i+blobcount]);
    }
    //
    pose_estimator.setImagePoints(image_points);
    //
    print_str(buf, &offset, len, "image_points:\n");
    for(int i=0; i < image_points.rows(); i++)
        print_str(buf, &offset, len, "%.3f\t", image_points(i).x());
    print_str(buf, &offset, len, "\n");
    for(int i=0; i < image_points.rows(); i++)
        print_str(buf, &offset, len, "%.3f\t", image_points(i).y());
    print_str(buf, &offset, len, "\n\n");



    print_str(buf, &offset, len, "Initial pose:\n");
    print_matrix(buf, &offset, len, pose_estimator.getPredictedPose());

    print_str(buf, &offset, len, "Max iter: %d\n\n", max_iter);
    pose_estimator.optimisePose(max_iter);

    Matrix4d optimized_pose;
    optimized_pose = pose_estimator.getPredictedPose();

    Rmat_mapped(0,0) = optimized_pose(0,0);
    Rmat_mapped(1,0) = optimized_pose(1,0);
    Rmat_mapped(2,0) = optimized_pose(2,0);
    Rmat_mapped(0,1) = optimized_pose(0,1);
    Rmat_mapped(1,1) = optimized_pose(1,1);
    Rmat_mapped(2,1) = optimized_pose(2,1);
    Rmat_mapped(0,2) = optimized_pose(0,2);
    Rmat_mapped(1,2) = optimized_pose(1,2);
    Rmat_mapped(2,2) = optimized_pose(2,2);

    tvec_mapped(0,0) = optimized_pose(0,3);
    tvec_mapped(1,0) = optimized_pose(1,3);
    tvec_mapped(2,0) = optimized_pose(2,3);

    print_str(buf, &offset, len, "Optimized pose:\n");
    print_matrix(buf, &offset, len, optimized_pose);

    print_str(buf, &offset, len, "Optimized pose covariance:\n");
    print_matrix(buf, &offset, len, pose_estimator.getPoseCovariance());

    print_str(buf, &offset, len, "Optimized Rmat:\n");
    print_matrix(buf, &offset, len, Rmat_mapped);
    print_str(buf, &offset, len, "Optimized tvec:\n");
    print_matrix(buf, &offset, len, tvec_mapped);

}




void print_str(char* buf, int* offset, size_t len, const char* fmt, ...)
{
    // if we use va_* macros, then we must also use the v*print* functions
    // see https://en.cppreference.com/w/cpp/utility/variadic

    va_list args;
    va_start(args, fmt);
    *offset += vsnprintf(buf + *offset, len - *offset, fmt, args);
    va_end(args);
}



void print_matrix(char* buf, int* offset, size_t len, const MatrixXd& m, const unsigned precision)
{

    char fmt[16];
    snprintf(fmt, 16, "%% .%df    ", precision);

    for (int r = 0; r < m.rows(); r++)
    {
        for (int c = 0; c < m.cols(); c++)
        {
            *offset += snprintf(buf + *offset, len - *offset, fmt, m(r, c));
        }
        *offset += snprintf(buf + *offset, len - *offset, "\n");
    }
    *offset += snprintf(buf + *offset, len - *offset, "\n");

}
