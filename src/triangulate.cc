/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "triangulate.h"
#include <math.h>
#include <sba.h>
#include <iostream>

/*
 * SBA parameter names:
 * m = number of images
 * n = number of points in 3D space
 * cnp = number of parameters for one camera (intrinsic + rotation +
 * translation)
 * pnp = number of parameters for one point (3 in our application X,Y,Z)
 * mnp = numper of parameters for one measurement (2 in our application i,j)
 */

namespace ggck {

/*
 * calcImgProj - calculates the projection of 3D point into image coordinates.
 * lifted from
 * SBA library's eucdemo project
 */
void calcImgProj(double a[5], double qr0[4], double v[3], double t[3],
                 double M[3], double n[2]) {
  double t1;
  double t10;
  double t12;
  double t14;
  double t16;
  double t18;
  double t19;
  double t2;
  double t25;
  double t26;
  double t3;
  double t32;
  double t33;
  double t35;
  double t36;
  double t4;
  double t42;
  double t46;
  double t5;
  double t51;
  double t52;
  double t57;
  double t58;
  double t6;
  double t69;
  double t7;
  double t77;
  double t80;
  double t9;
  {
    t1 = a[0];
    t2 = v[0];
    t3 = t2 * t2;
    t4 = v[1];
    t5 = t4 * t4;
    t6 = v[2];
    t7 = t6 * t6;
    t9 = sqrt(1.0 - t3 - t5 - t7);
    t10 = qr0[1];
    t12 = qr0[0];
    t14 = qr0[3];
    t16 = qr0[2];
    t18 = t9 * t10 + t12 * t2 + t4 * t14 - t6 * t16;
    t19 = M[0];
    t25 = t9 * t16 + t12 * t4 + t6 * t10 - t2 * t14;
    t26 = M[1];
    t32 = t9 * t14 + t12 * t6 + t2 * t16 - t4 * t10;
    t33 = M[2];
    t35 = -t18 * t19 - t25 * t26 - t32 * t33;
    t36 = -t18;
    t42 = t9 * t12 - t2 * t10 - t4 * t16 - t6 * t14;
    t46 = t42 * t19 + t25 * t33 - t32 * t26;
    t51 = t42 * t26 + t32 * t19 - t18 * t33;
    t52 = -t32;
    t57 = t42 * t33 + t18 * t26 - t25 * t19;
    t58 = -t25;
    t69 = t35 * t58 + t42 * t51 + t57 * t36 - t46 * t52 + t[1];
    t77 = t35 * t52 + t42 * t57 + t46 * t58 - t51 * t36 + t[2];
    t80 = 1 / t77;
    n[0] = (t1 * (t35 * t36 + t42 * t46 + t51 * t52 - t57 * t58 + t[0]) +
            a[4] * t69 + a[1] * t77) *
           t80;
    n[1] = (t1 * a[3] * t69 + a[2] * t77) * t80;
    return;
  }
}

/*
 * Reprojection function for use with SBA. From the SBA Library:
 * -------------------------------------------------------------------------
 * functional relation computing a SINGLE image measurement. Assuming that
 * the parameters of point i are bi and the parameters of camera j aj,
 * computes a prediction of \hat{x}_{ij}. aj is cnp x 1, bi is pnp x 1 and
 * xij is mnp x 1. This function is called only if point i is visible in
 * image j (i.e. vmask[i, j]==1)
 * -------------------------------------------------------------------------
 * lifted from the SBA library's eucdemo project
 */
void reproject(int j, int i, double *aj, double *bi, double *xij, void *adata) {
  double *pr0;
  struct globs_ *gl;

  gl = (struct globs_ *)adata;
  // pr0 = gl->rot0params + j * FULLQUATSZ; // full quat for initial rotation
  // estimate

  calcImgProj(
      aj, pr0, aj + 5, aj + 5 + 3, bi,
      xij);  // 5 for the calibration + 3 for the quaternion's vector part
}

/*
 * Jacobian calculation. From the SBA Libarary:
 * ------------------------------------------------------------------------------
 * function to evaluate the sparse Jacobian dX/dp.
 * The Jacobian is returned in jac as
 * (dx_11/da_1, ..., dx_1m/da_m, ..., dx_n1/da_1, ..., dx_nm/da_m,
 *  dx_11/db_1, ..., dx_1m/db_1, ..., dx_n1/db_n, ..., dx_nm/db_n), or
 * (using HZ's notation),
 * jac=(A_11, B_11, ..., A_1m, B_1m, ..., A_n1, B_n1, ..., A_nm, B_nm)
 * Notice that depending on idxij, some of the A_ij and B_ij might be missing.
 * Note also that A_ij and B_ij are mnp x cnp and mnp x pnp matrices resp. and
 * should be stored in jac in row-major order.
 * rcidxs, rcsubs are max(m, n) x 1, allocated by the caller and can be used
 * as working memory
 */
void jacobian(double *p, struct sba_crsm *idxij, int *rcidxs, int *rcsubs,
              double *jac, void *adata) {}

}  // namespace ggck
