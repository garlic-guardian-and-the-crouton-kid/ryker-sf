#include "sba_utils.h"
#include <sba.h>
#include <math.h>

/*
 * SBA parameter names:
 * m = number of images
 * n = number of points in 3D space
 * cnp = number of parameters for one camera (intrinsic + rotation + translation)
 * pnp = number of parameters for one point (3 in our application X,Y,Z)
 * mnp = numper of parameters for one measurement (2 in our application i,j)
 */

namespace ggck {

struct camera_params {
  double f;     // focal length of camera
  double cx;    // x center of camera (image coordinates)
  double cy;    // y center of camera (image coordinates)
  double roll;  // camera roll
  double pitch; // camera pitch
  double yaw;   // camera camera yaw
  double tx;    // camera x translation
  double ty;    // camera y translation
  double tz;    // camera z translation
};


void MkQuatFrmVec(double * q, double * v) {
  (q)[1] = (v)[0];
  (q)[2] = (v)[1];
  (q)[3] = (v)[2];
  (q)[0] = sqrt(1.0 - (q)[1] * (q)[1] - (q)[2] * (q)[2] - (q)[3] * (q)[3]);
}


/* convert a vector of camera parameters so that rotation is represented by
* the vector part of the input quaternion. The function converts the
* input quaternion into a unit one with a non-negative scalar part. Remaining
* parameters are left unchanged.
*
* Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
* Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
* Lifted from SBA eucdemo project
*/
void quat2vec(double *inp, int nin, double *outp, int nout)
{
  double mag, sg;
  register int i;

  /* intrinsics & distortion */
  if (nin>7) // are they present?
    for (i = 0; i<nin - 7; ++i)
      outp[i] = inp[i];
  else
    i = 0;

  /* rotation */
  /* normalize and ensure that the quaternion's scalar component is non-negative;
  * if not, negate the quaternion since two quaternions q and -q represent the
  * same rotation
  */
  mag = sqrt(inp[i] * inp[i] + inp[i + 1] * inp[i + 1] + inp[i + 2] * inp[i + 2] + inp[i + 3] * inp[i + 3]);
  sg = (inp[i] >= 0.0) ? 1.0 : -1.0;
  mag = sg / mag;
  outp[i] = inp[i + 1] * mag;
  outp[i + 1] = inp[i + 2] * mag;
  outp[i + 2] = inp[i + 3] * mag;
  i += 3;

  /* translation*/
  for (; i<nout; ++i)
    outp[i] = inp[i + 1];
}

/* convert a vector of camera parameters so that rotation is represented by
* a full unit quaternion instead of its input 3-vector part. Remaining
* parameters are left unchanged.
*
* Input parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion vector part (3), translation (3)
* Output parameter layout: intrinsics (5, optional), distortion (5, optional), rot. quaternion (4), translation (3)
* Lifted from SBA eucdemo project
*/
void vec2quat(double *inp, int nin, double *outp, int nout)
{
  double *v, q[4];
  register int i;

  /* intrinsics & distortion */
  if (nin>7 - 1) // are they present?
    for (i = 0; i<nin - (7 - 1); ++i)
      outp[i] = inp[i];
  else
    i = 0;

  /* rotation */
  /* recover the quaternion from the vector */
  v = inp + i;
  MkQuatFrmVec(q, v);
  outp[i] = q[0];
  outp[i + 1] = q[1];
  outp[i + 2] = q[2];
  outp[i + 3] = q[3];
  i += 4;

  /* translation */
  for (; i<nout; ++i)
    outp[i] = inp[i - 1];
}

} //namespace ggck