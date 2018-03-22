/*
* Utility functions associated with the SBA libary.
* Heavily based on the SBA sample project eucdemo
*/
#ifndef INCLUDE_SBA_UTILS_H_
#define INCLUDE_SBA_UTILS_H_

namespace ggck {

/* convert a vector of camera parameters so that rotation is represented by
 * the vector part of the input quaternion. The function converts the
 * input quaternion into a unit one with a non-negative scalar part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 * Lifted from SBA eucdemo project
 */
void quat2vec(double *inp, int nin, double *outp, int nout);

/* convert a vector of camera parameters so that rotation is represented by
 * a full unit quaternion instead of its input 3-vector part. Remaining
 * parameters are left unchanged.
 *
 * Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
 * Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
 * Lifted from SBA eucdemo project
 */
void vec2quat(double *inp, int nin, double *outp, int nout);

} //namespace ggck

#endif //INCLUDE_SBA_UTILS_H_