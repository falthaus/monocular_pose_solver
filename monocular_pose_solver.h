/*
 * monocular_pose_solver.h
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


#pragma once

#ifdef DLL_EXPORTS
#define DLL_API __declspec(dllexport)
#else
#define DLL_API __declspec(dllimport)
#endif



/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API void set_camera_matrix(double cm[3*3], char* buf, size_t len);



/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API void set_world_points(double wp[3*4], char* buf, size_t len);


/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API int solve_p4p(float blobs[2*4], double Rmat[3*3], double tvec[3*1], float* reprojection_error, char* buf, size_t len);


/*
 *
 *
 *
 *
 *
 */
//extern "C" DLL_API void solve_pnp(float* blobs, unsigned int blobcount, double* Rmat, double* tvec, float* reprojection_error, char* buf, size_t len);


/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API void optimize_pose(float* blobs, unsigned blobcount, unsigned* correspondences, double* Rmat, double* tvec, char* buf, size_t len, unsigned max_iter);
