/*
 * Copyright 2018 Justin Manley and Joseph Bolling.
 */
#include "triangulate.h"
#include "sba_utils.h"
#include "point_set.h"


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

/* pointers to additional data, used for computed image projections and their jacobians 
 * lifted from SBA's eucdemo project
 */
struct globs_ {
  double *rot0params; /* initial rotation parameters, combined with a local rotation parameterization */
  double *intrcalib; /* the 5 intrinsic calibration parameters in the order [fu, u0, v0, ar, skew],
                      * where ar is the aspect ratio fv/fu.
                      * Used only when calibration is fixed for all cameras;
                      * otherwise, it is null and the intrinsic parameters are
                      * included in the set of motion parameters for each camera
                      */
  int nccalib; /* number of calibration parameters that must be kept constant.
                * 0: all parameters are free
                * 1: skew is fixed to its initial value, all other parameters vary (i.e. fu, u0, v0, ar)
                * 2: skew and aspect ratio are fixed to their initial values, all other parameters vary (i.e. fu, u0, v0)
                * 3: meaningless
                * 4: skew, aspect ratio and principal point are fixed to their initial values, only the focal length varies (i.e. fu)
                * 5: all intrinsics are kept fixed to their initial values
                * >5: meaningless
                * Used only when calibration varies among cameras
                */

  int ncdist; /* number of distortion parameters in Bouguet's model that must be kept constant.
              * 0: all parameters are free
              * 1: 6th order radial distortion term (kc[4]) is fixed
              * 2: 6th order radial distortion and one of the tangential distortion terms (kc[3]) are fixed
              * 3: 6th order radial distortion and both tangential distortion terms (kc[3], kc[2]) are fixed [i.e., only 2nd & 4th order radial dist.]
              * 4: 4th & 6th order radial distortion terms and both tangential distortion ones are fixed [i.e., only 2nd order radial dist.]
              * 5: all distortion parameters are kept fixed to their initial values
              * >5: meaningless
              * Used only when calibration varies among cameras and distortion is to be estimated
              */
  int cnp, pnp, mnp; /* dimensions */

  double *ptparams; /* needed only when bundle adjusting for camera parameters only */
  double *camparams; /* needed only when bundle adjusting for structure parameters only */
} globs;

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
 * Jacobian calculation
 * lifted from SBA library eucdemo project
 */
void calcImgProjJacKRTS(double a[5], double qr0[4], double v[3], double t[3],
  double M[3], double jacmKRT[2][11], double jacmS[2][3])
{
  double t1;
  double t102;
  double t107;
  double t109;
  double t11;
  double t114;
  double t116;
  double t120;
  double t129;
  double t13;
  double t131;
  double t140;
  double t148;
  double t149;
  double t15;
  double t150;
  double t152;
  double t154;
  double t161;
  double t164;
  double t167;
  double t17;
  double t170;
  double t172;
  double t174;
  double t177;
  double t18;
  double t182;
  double t187;
  double t189;
  double t194;
  double t196;
  double t2;
  double t208;
  double t218;
  double t229;
  double t232;
  double t235;
  double t237;
  double t239;
  double t24;
  double t242;
  double t247;
  double t25;
  double t252;
  double t254;
  double t259;
  double t261;
  double t273;
  double t283;
  double t295;
  double t296;
  double t298;
  double t3;
  double t301;
  double t304;
  double t305;
  double t307;
  double t308;
  double t31;
  double t311;
  double t32;
  double t326;
  double t327;
  double t329;
  double t332;
  double t333;
  double t34;
  double t349;
  double t35;
  double t352;
  double t4;
  double t41;
  double t45;
  double t5;
  double t50;
  double t51;
  double t56;
  double t57;
  double t6;
  double t60;
  double t66;
  double t67;
  double t68;
  double t74;
  double t76;
  double t78;
  double t79;
  double t8;
  double t81;
  double t83;
  double t85;
  double t87;
  double t89;
  double t9;
  double t91;
  double t93;
  double t95;
  double t97;
  {
    t1 = v[0];
    t2 = t1 * t1;
    t3 = v[1];
    t4 = t3 * t3;
    t5 = v[2];
    t6 = t5 * t5;
    t8 = sqrt(1.0 - t2 - t4 - t6);
    t9 = qr0[1];
    t11 = qr0[0];
    t13 = qr0[3];
    t15 = qr0[2];
    t17 = t8 * t9 + t11 * t1 + t13 * t3 - t5 * t15;
    t18 = M[0];
    t24 = t8 * t15 + t3 * t11 + t5 * t9 - t13 * t1;
    t25 = M[1];
    t31 = t8 * t13 + t5 * t11 + t1 * t15 - t3 * t9;
    t32 = M[2];
    t34 = -t17 * t18 - t24 * t25 - t31 * t32;
    t35 = -t17;
    t41 = t11 * t8 - t1 * t9 - t3 * t15 - t5 * t13;
    t45 = t41 * t18 + t24 * t32 - t31 * t25;
    t50 = t41 * t25 + t31 * t18 - t17 * t32;
    t51 = -t31;
    t56 = t41 * t32 + t17 * t25 - t24 * t18;
    t57 = -t24;
    t60 = t34 * t35 + t41 * t45 + t50 * t51 - t56 * t57 + t[0];
    t66 = t34 * t51 + t41 * t56 + t45 * t57 - t50 * t35 + t[2];
    t67 = 1 / t66;
    jacmKRT[0][0] = t60 * t67;
    t68 = a[3];
    t74 = t34 * t57 + t41 * t50 + t56 * t35 - t45 * t51 + t[1];
    jacmKRT[1][0] = t68 * t74*t67;
    jacmKRT[0][1] = 1.0;
    jacmKRT[1][1] = 0.0;
    jacmKRT[0][2] = 0.0;
    jacmKRT[1][2] = 1.0;
    jacmKRT[0][3] = 0.0;
    t76 = a[0];
    jacmKRT[1][3] = t76 * t74*t67;
    jacmKRT[0][4] = t74 * t67;
    jacmKRT[1][4] = 0.0;
    t78 = 1 / t8;
    t79 = t78 * t9;
    t81 = -t79 * t1 + t11;
    t83 = t78 * t15;
    t85 = -t83 * t1 - t13;
    t87 = t78 * t13;
    t89 = -t87 * t1 + t15;
    t91 = -t81 * t18 - t85 * t25 - t89 * t32;
    t93 = -t81;
    t95 = t78 * t11;
    t97 = -t95 * t1 - t9;
    t102 = t97 * t18 + t85 * t32 - t89 * t25;
    t107 = t97 * t25 + t89 * t18 - t81 * t32;
    t109 = -t89;
    t114 = t97 * t32 + t81 * t25 - t85 * t18;
    t116 = -t85;
    t120 = a[4];
    t129 = t91 * t57 + t34 * t116 + t97 * t50 + t41 * t107 + t114 * t35 + t56 * t93 - t102 * t51 - t45 * t109
      ;
    t131 = a[1];
    t140 = t91 * t51 + t34 * t109 + t97 * t56 + t41 * t114 + t102 * t57 + t45 * t116 - t107 * t35 - t50 * t93
      ;
    t148 = t66 * t66;
    t149 = 1 / t148;
    t150 = (t76*t60 + t120 * t74 + t131 * t66)*t149;
    jacmKRT[0][5] = (t76*(t91*t35 + t34 * t93 + t97 * t45 + t41 * t102 + t107 * t51 + t50 * t109 -
      t114 * t57 - t56 * t116) + t129 * t120 + t131 * t140)*t67 - t150 * t140;
    t152 = t76 * t68;
    t154 = a[2];
    t161 = (t152*t74 + t154 * t66)*t149;
    jacmKRT[1][5] = (t152*t129 + t154 * t140)*t67 - t161 * t140;
    t164 = -t79 * t3 + t13;
    t167 = -t83 * t3 + t11;
    t170 = -t87 * t3 - t9;
    t172 = -t164 * t18 - t167 * t25 - t170 * t32;
    t174 = -t164;
    t177 = -t95 * t3 - t15;
    t182 = t177 * t18 + t167 * t32 - t170 * t25;
    t187 = t177 * t25 + t170 * t18 - t164 * t32;
    t189 = -t170;
    t194 = t177 * t32 + t164 * t25 - t167 * t18;
    t196 = -t167;
    t208 = t172 * t57 + t34 * t196 + t177 * t50 + t41 * t187 + t194 * t35 + t56 * t174 - t182 * t51 - t45 *
      t189;
    t218 = t172 * t51 + t34 * t189 + t177 * t56 + t41 * t194 + t182 * t57 + t45 * t196 - t187 * t35 - t50 *
      t174;
    jacmKRT[0][6] = (t76*(t172*t35 + t34 * t174 + t177 * t45 + t41 * t182 + t187 * t51 + t50 * t189
      - t194 * t57 - t56 * t196) + t120 * t208 + t131 * t218)*t67 - t150 * t218;
    jacmKRT[1][6] = (t152*t208 + t154 * t218)*t67 - t161 * t218;
    t229 = -t79 * t5 - t15;
    t232 = -t83 * t5 + t9;
    t235 = -t87 * t5 + t11;
    t237 = -t229 * t18 - t232 * t25 - t235 * t32;
    t239 = -t229;
    t242 = -t95 * t5 - t13;
    t247 = t242 * t18 + t232 * t32 - t235 * t25;
    t252 = t242 * t25 + t235 * t18 - t229 * t32;
    t254 = -t235;
    t259 = t242 * t32 + t229 * t25 - t232 * t18;
    t261 = -t232;
    t273 = t237 * t57 + t261 * t34 + t242 * t50 + t41 * t252 + t259 * t35 + t56 * t239 - t247 * t51 - t45 *
      t254;
    t283 = t237 * t51 + t34 * t254 + t242 * t56 + t41 * t259 + t247 * t57 + t45 * t261 - t252 * t35 - t50 *
      t239;
    jacmKRT[0][7] = (t76*(t237*t35 + t34 * t239 + t242 * t45 + t41 * t247 + t252 * t51 + t50 * t254
      - t259 * t57 - t56 * t261) + t120 * t273 + t131 * t283)*t67 - t150 * t283;
    jacmKRT[1][7] = (t152*t273 + t154 * t283)*t67 - t161 * t283;
    jacmKRT[0][8] = t76 * t67;
    jacmKRT[1][8] = 0.0;
    jacmKRT[0][9] = t120 * t67;
    jacmKRT[1][9] = t152 * t67;
    jacmKRT[0][10] = t131 * t67 - t150;
    jacmKRT[1][10] = t154 * t67 - t161;
    t295 = t35 * t35;
    t296 = t41 * t41;
    t298 = t57 * t57;
    t301 = t35 * t57;
    t304 = t41 * t51;
    t305 = 2.0*t301 + t41 * t31 - t304;
    t307 = t35 * t51;
    t308 = t41 * t57;
    t311 = t307 + 2.0*t308 - t31 * t35;
    jacmS[0][0] = (t76*(t295 + t296 + t31 * t51 - t298) + t120 * t305 + t131 * t311)*t67 - t150 *
      t311;
    jacmS[1][0] = (t152*t305 + t154 * t311)*t67 - t161 * t311;
    t326 = t51 * t51;
    t327 = t298 + t296 + t17 * t35 - t326;
    t329 = t57 * t51;
    t332 = t41 * t35;
    t333 = 2.0*t329 + t41 * t17 - t332;
    jacmS[0][1] = (t76*(t301 + 2.0*t304 - t17 * t57) + t120 * t327 + t131 * t333)*t67 - t150 *
      t333;
    jacmS[1][1] = (t152*t327 + t154 * t333)*t67 - t161 * t333;
    t349 = t329 + 2.0*t332 - t24 * t51;
    t352 = t326 + t296 + t24 * t57 - t295;
    jacmS[0][2] = (t76*(2.0*t307 + t41 * t24 - t308) + t120 * t349 + t131 * t352)*t67 - t150 *
      t352;
    jacmS[1][2] = (t152*t349 + t154 * t352)*t67 - t161 * t352;
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
  double pr0[4];
  struct globs_ *gl;

  // hardcode initial rotation array.
  // TODO(jbolling) update so that initila rotations are actually carried through from PointSet
  pr0[0] = 1;
  pr0[1] = 0;
  pr0[2] = 0;
  pr0[3] = 0;

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
void jacobian(int j, int i, double *aj, double *bi, double *Aij, double *Bij, 
              void *adata) {
  struct globs_ *gl;
  double pr0[4];
  int ncK;

  gl = (struct globs_ *)adata;
  // hardcode initial rotation array.
  // TODO(jbolling) update so that initila rotations are actually carried through from PointSet
  pr0[0] = 1;
  pr0[1] = 0;
  pr0[2] = 0;
  pr0[3] = 0;

  calcImgProjJacKRTS(aj, pr0, aj + 5, aj + 5 + 3, bi, (double(*)[5 + 6])Aij, (double(*)[3])Bij); // 5 for the calibration + 3 for the quaternion's vector part

                                                                                                 /* clear the columns of the Jacobian corresponding to fixed calibration parameters */
  gl = (struct globs_ *)adata;
  ncK = gl->nccalib;
  if (ncK) {
    int cnp, mnp, j0;

    cnp = gl->cnp;
    mnp = gl->mnp;
    j0 = 5 - ncK;

    for (i = 0; i<mnp; ++i, Aij += cnp)
      for (j = j0; j<5; ++j)
        Aij[j] = 0.0; // Aij[i*cnp+j]=0.0;
  }
}

std::vector<cv::Point3d> RunSba(PointSet* pointSet)
{
  int cnp = 11;
  int pnp = 3;
  int mnp = 2;
  int numPoints3D = pointSet->Num3DPoints();
  int numFrames = pointSet->NumFrames();
  int verbose = 0;
  int maxIter = 150;
  double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
  /* I: minim. options [\mu, \epsilon1, \epsilon2, \epsilon3, \epsilon]. Respectively the scale factor for initial \mu,
  * stoping thresholds for ||J^T e||_inf, ||dp||_2, ||e||_2 and (||e||_2-||e_new||_2)/||e||_2
  */
  opts[0] = SBA_INIT_MU; 
  opts[1] = SBA_STOP_THRESH; 
  opts[2] = 1E-18;
  opts[3] = SBA_STOP_THRESH;
  //opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
  opts[4] = 0.0;

  std::vector<double>initParams = pointSet->GetSbaInitialParams(cnp);
  SbaMeasurementInfo measurementInfo = pointSet->GetSbaMeasurementInfo();

  /* set up globs structure */
  globs.cnp = cnp; globs.pnp = pnp; globs.mnp = mnp;
  globs.rot0params = NULL;
  globs.intrcalib = NULL;
  /* specify the number of intrinsic parameters that are to be fixed
  * equal to their initial values, as follows:
  * 0: all free, 1: skew fixed, 2: skew, ar fixed, 4: skew, ar, ppt fixed
  * Note that a value of 3 does not make sense
  */
  globs.nccalib = 0; /* number of intrinsics to keep fixed, must be between 0 and 5 */
  globs.ncdist = -9999;

  globs.ptparams = NULL;
  globs.camparams = NULL;

  double reprojection[2];
  reproject(0, 0, &initParams[0], &initParams[numFrames*cnp], reprojection, (void*)(&globs));
  std::printf("f: %f, u0: %f, v0: %f, ar: %f, s: %f, r0: %f, r1: %f, r2: %f, t0: %f, t1: %f, t2: %f \n",
    initParams[0], initParams[1], initParams[2], initParams[3],
    initParams[4], initParams[5], initParams[6], initParams[7], initParams[8], initParams[9], initParams[10]);
  std::printf("point x: %f, point y: %f, point z: %f \n", 
    initParams[numFrames*cnp], initParams[numFrames*cnp + 1], initParams[numFrames*cnp + 2]);
  std::printf("Reprojection x: %f, Reprojection y: %f \n", reprojection[0], reprojection[1]);
  std::printf("Original x: %f, Original y: %f \n", measurementInfo.measurements[0], measurementInfo.measurements[1]);

  int n = sba_motstr_levmar(numPoints3D, 0, numFrames, 0, &measurementInfo.vmask[0], &initParams[0],
    cnp, pnp, &measurementInfo.measurements[0], NULL, mnp, reproject, jacobian,
    (void *)(&globs), maxIter , verbose, opts, info);
  
  fprintf(stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n", n,
    info[5], info[6], info[1], info[0], (int)info[7], (int)info[8], (int)info[9]);

  std::vector<cv::Point3d> pointCloud(numPoints3D);
  int pIndex = numFrames * cnp;
  // iterate over returned values to put back into a structured array
  for (auto & point : pointCloud)
  {
    point.x = initParams[pIndex];
    point.y = initParams[pIndex + 1];
    point.z = initParams[pIndex + 2];
    pIndex += mnp;
  }
  return pointCloud;
}

}  // namespace ggck
