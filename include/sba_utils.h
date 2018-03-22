/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 *
 * Utility functions associated with the SBA libary.
 * Heavily based on the SBA sample project eucdemo
 */
#ifndef INCLUDE_SBA_UTILS_H_
#define INCLUDE_SBA_UTILS_H_

namespace ggck {

// Struct with camera parameters for cnp = 11
// This struct uses the quaternion description of rotation, and so has an
// extra degree of freedom (q3). It is reduced to 11 DoF using the quat2vec
// function
struct CameraParams {
  double f;   // focal length
  double u0;  // center of the image in x
  double v0;  // center of the image in y
  double ar;  // aspect ratio between x and y focal lengths
  double s;   // skewness factor for non-square pixels
  double q0;  // quaternion scalar element
  double q1;  // quaternion second element
  double q2;  // quaternion third element
  double q3;  // quaternion fourth element
  double x;   // translation in x
  double y;   // translation in y
  double z;   // translation in z
};

/* convert a vector of camera parameters so that rotation is represented by
 * the vector part of the input quaternion. The function converts the
 * input quaternion into a unit one with a non-negative scalar part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional),
 * rot. quaternion (4), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional),
 * rot. quaternion vector part (3), translation (3)
 * Lifted from SBA eucdemo project
 */
void quat2vec(double *inp, int nin, double *outp, int nout);

/* convert a vector of camera parameters so that rotation is represented by
 * a full unit quaternion instead of its input 3-vector part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional),
 * rot. quaternion vector part (3), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional),
 * rot. quaternion (4), translation (3)
 * Lifted from SBA eucdemo project
 */
void vec2quat(double *inp, int nin, double *outp, int nout);

}  // namespace ggck

#endif  // INCLUDE_SBA_UTILS_H_
