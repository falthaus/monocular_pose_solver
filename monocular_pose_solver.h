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
extern "C" DLL_API void set_camera_matrix(double* cm, char* buf, size_t len);



/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API void set_world_points(double* wp, char* buf, size_t len);


/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API void solve_p4p(float* blobs, double* R, double* t, float* reprojection_error, char* buf, size_t len);


/*
 *
 *
 *
 *
 *
 */
extern "C" DLL_API void solve_pnp(float* blobs, unsigned int blobcount, double* R, double* t, float* reprojection_error, char* buf, size_t len);
