import argparse
import matplotlib.pyplot as plt
import numpy as np
from osgeo import gdal
import osr
import pyproj
import sys


'''
GET_PROJECTION
Arguments:
    gdal_dataset - A GDALDataset object.

Returns:
    A Proj4 object representing the projection of the gdal_dataset.
'''
def get_projection(gdal_dataset):
    proj_wkt = gdal_dataset.GetProjection()
    proj_converter = osr.SpatialReference()
    proj_converter.ImportFromWkt(proj_wkt)
    return pyproj.Proj(proj_converter.ExportToProj4())


'''
APPLY_GEO_TRANSFORM Apply a translation and scaling to a set of numbers.
Arguments:
    coords - A scalar or 1-dimensional ndarray

    translation - A scalar indicating the amount to translate
    
    scale - A scalar indicating the amount to scale

Returns:
    A scalar or 1-dimensional ndarray
'''
def apply_geo_transform(coords, translation, scale):
    return translation + scale * coords


'''
GEO_TRANSFORM_X Apply an affine transform mapping x-coordinates in image
   space to x-coordinates in projected geographic space. Note that we can
   ignore some of the parameters of the affine transform because we are
   assuming that we are transforming along the x-axis, which means that
   y = 0, so we can ignore the y-parameters in the affine transform.

Arguments:
    coords - Scalar or 1-dimensional ndarray of x-coordinates.

    geo_transform - Array of size 6 containing the parameters of a geo
        transform. See GDAL documentation.
Returns:
    A scalar or 1-dimensional ndarray of the transformed x-coordinates.
'''
def geo_transform_x(coords, geo_transform):
    return apply_geo_transform(coords, geo_transform[0], geo_transform[1])


'''
GEO_TRANSFORM_Y Apply an affine transform mapping y-coordinates in image
   space to y-coordinates in projected geographic space. Note that we can
   ignore some of the parameters of the affine transform because we are
   assuming that we are transforming along the y-axis, which means that
   x = 0, so we can ignore the x-parameters in the affine transform.

Arguments:
    coords - Scalar or 1-dimensional ndarray of y-coordinates.

    geo_transform - Array of size 6 containing the parameters of a geo
        transform. See GDAL documentation.
Returns:
    A scalar or 1-dimensional ndarray of the transformed y-coordinates.
'''
def geo_transform_y(coords, geo_transform):
    return apply_geo_transform(coords, geo_transform[3], geo_transform[5])


'''
GET_PROJECTED_ESTIMATES Project the estimates into the coordinate reference
    system (CRS) of the ground truth elevations.
Arguments:
    estimates - (P, 3) ndarray of x,y,z coordinates.

    estimates_projection - WKT string indicating the CRS for estimates.

    ground_truth_dataset - A GDALDataset representing the ground truth

    ground_truth_elevations - An ndarray representing the ground truth

Returns:
    A (P, 3) ndarray of x,y,z coordinates in the coordinate reference system
    of the ground truth elevations.
'''
def get_projected_estimates(
        estimates, estimates_projection,
        ground_truth_dataset, ground_truth_elevations):
    estimate_projection = pyproj.Proj(estimates_projection)
    ground_truth_projection = get_projection(ground_truth_dataset)
    ground_truth_geo_transform = ground_truth_dataset.GetGeoTransform()

    def project_estimate_to_ground_truth(p):
        return pyproj.transform(
            estimate_projection,
            ground_truth_projection,
            p[0], p[1], p[2])

    # Project the estimates points into the coordinate reference system of
    # the DEM.
    estimates_projected = np.apply_along_axis(
        project_estimate_to_ground_truth, 1, estimates)

    ground_truth_x_bounds = np.sort(
        geo_transform_x(np.array([0, ground_truth_elevations.shape[1]]),
            ground_truth_geo_transform))
    ground_truth_y_bounds = np.sort(
        geo_transform_y(np.array([0, ground_truth_elevations.shape[0]]),
            ground_truth_geo_transform))

    # Somehow our reconstruction reversed the sign of the y-coordinates.
    # TODO(justinmanley): Figure out where in the reconstruction pipeline the
    # sign gets reversed.
    estimates_projected[:,1] *= -1

    # All of the reconstructed points should fall within the xy bounds of the
    # digital elevation map.
    within_x_bounds = np.logical_and(
        ground_truth_x_bounds[0] < estimates_projected[:,0],
        estimates_projected[:,0] < ground_truth_x_bounds[1])
    within_y_bounds = np.logical_and(
        ground_truth_y_bounds[0] < estimates_projected[:,1],
        estimates_projected[:,1] < ground_truth_y_bounds[1])
    within_bounds = np.logical_and(within_x_bounds, within_y_bounds)
    assert(np.all(within_bounds))

    return estimates_projected


'''
GET_CORRESPONDING_GROUND_TRUTH_POINTS
Arguments:
    estimates - A (K, 3) ndarray, where K is the number of reconstructed points

    ground_truth_dataset - A GDALDataset containing the ground truth elevation data

    ground_truth_elevations - An (M, N) ndarray in which each value indicates the
        elevation at the (i, j)-th bin in the raster.

Returns:
    A tuple containing the x-indices and the y-indices, respectively, for the
    bins in ground_truth_elevations corresponding to the points in estimates. That
    is, this function returns a tuple (xs, ys) such that (xs[i], ys[i]) is the
    2D index into ground_truth_elevations corresponding to the ith point in
    estimates.
'''
def get_corresponding_ground_truth_points(estimates, ground_truth_dataset, ground_truth_elevations):
    ground_truth_geo_transform = ground_truth_dataset.GetGeoTransform()

    # The coordinates of bins in the geographic coordinate system of the ground
    # truth dataset. The first element of this tuple contains the x-coordinates
    # of the bins; the second element of this  tuple contains the y-coordinates.
    bins = (
        geo_transform_x(np.arange(ground_truth_elevations.shape[1]),
            ground_truth_geo_transform),
        geo_transform_y(np.arange(ground_truth_elevations.shape[0]),
            ground_truth_geo_transform))

    # Flip x and y-coordinates to account for the difference in representation
    # as rows/columns vs. width/height.
    indices = (
        np.digitize(estimates[:,1], bins[1]),
        np.digitize(estimates[:,0], bins[0]))

    return indices

    # Reconstruction using structure-from-motion (e.g. bundle adjustment) is
    # only up to an affine transformation, so we fit an affine transformation
    # to align the estimates with the ground truth before evaluating the
    # estimates. In our pipeline, bundle adjustment does not seem to affect
    # the x and y coordinates, so we only fit an affine transformation on the
    # elevations.

'''
GET_ESTIMATE_TO_GROUND_TRUTH_AFFINE_MAP Compute an affine map which maps the
estimated points onto the ground truth. Reconstruction using
structure-from-motion (e.g. bundle adjustment) is only up to an affine
transformation, so we fit an affine transformation to align the estimates with
the ground truth before evaluating the estimates. We only fit an affine map for
the elevations, since we assume that the x and y coordinates are identical
between the ground truth and the reconstruction.
Arguments:
    estimated_elevations - A (P, 3) ndarray containing the x,y,z coordinates of
        each estimated point.

    ground_truth_elevations - An (M, N) ndarray containing the ground truth
        elevations.

    ground_truth_dataset - A GDALDataset representing the ground truth

    ground_truth_indices - A tuple containing two (P,)-shaped ndarrays with the
        indices (x and y, respectively), of the bins in the ground truth DEM
        corresponding to each estimated point.

Returns:
    A tuple of (A, b) such that Ae + b = g, where e = (x, y, z) is an estimated
    point, and g = (x, y, z) is a ground truth point.

    scaling_matrix - A (3,3) ndarray.

    translation_vector -  A (3, 1) ndarray.
'''
def get_estimate_to_ground_truth_affine_map(
        estimates, ground_truth_elevations, ground_truth_dataset, ground_truth_indices):
    ground_truth_geo_transform = ground_truth_dataset.GetGeoTransform()

    # The equation to be solved by least-squares has the form:
    #     [ x y z 1 ]   [ w ]
    # for each point, where (x, y, z) is an estimated point, and
    # (u, v, w) is the corresponding ground truth point.
    estimates_measurements = np.hstack((estimates, np.ones((estimates.shape[0], 1))))

    ground_truth_measurements = ground_truth_elevations[ground_truth_indices].flatten()

    affine_map_parameters, _, _, _ = np.linalg.lstsq(
        estimates_measurements,
        ground_truth_measurements,
        rcond = None)

    return affine_map_parameters[0:3], affine_map_parameters[3]

'''
EXPORT_POINTS_FOR_RENDERING Write estimated and ground truth points to csv files
suitable for use in Meshlab.
Arguments:
    estimates - A (P, 3) ndarray containing the x,y,z coordinates of each
        estimated point.

    ground_truth_elevations - An (M, N) ndarray containing the ground truth
        elevations.
        
    ground_truth_dataset - A GDALDataset representing the ground truth

    ground_truth_indices - A tuple containing two (P,)-shaped ndarrays with the
        indices (x and y, respectively), of the bins in the ground truth DEM
        corresponding to each estimated point.

Returns:
    None
'''
def export_points_for_rendering(
        estimates, ground_truth_elevations,
        ground_truth_dataset, ground_truth_indices):
    # Ball pivoting surface reconstruction in Meshlab only works when the
    # range of data on the x, y, and z axes are all similar.
    # The x and y coordinates in the ground truth data set are in the same
    # units, while the z coordinates are in different units (meters).
    # Scaling the z coordinates to match the range of the x coordinates is
    # a hack to get ball-pivoting surface reconstruction to work in Meshlab.
    estimates_x_range = np.max(estimates[:,0]) - np.min(estimates[:,0])
    ground_truth_elevations_range = (
        np.max(ground_truth_elevations) - np.min(ground_truth_elevations))
    scaled_ground_truth_elevations = ground_truth_elevations * (
        estimates_x_range / (5 * ground_truth_elevations_range))

    ground_truth_matches = np.hstack((
        estimates[:,0:2],
        scaled_ground_truth_elevations[ground_truth_indices].reshape(
            len(ground_truth_indices[0]), 1)))
    np.savetxt("selected_ground_truth_points.asc", ground_truth_matches,
        delimiter = ',')

    # Align the estimates to the ground truth data with an affine
    # transformation.
    scaling_matrix, translation_vector = get_estimate_to_ground_truth_affine_map(
        estimates, scaled_ground_truth_elevations,
        ground_truth_dataset, ground_truth_indices)
    affine_estimated_elevations = np.hstack((
        estimates[:,0:2],
        (scaling_matrix.dot(estimates.T).T + translation_vector).reshape(
            estimates.shape[0], 1)))
    np.savetxt("affine_adjusted_points.asc", affine_estimated_elevations,
        delimiter=',')


'''
COMPUTE_RMSE Compute root mean square error (RMSE) between elevation estimates
and ground truth.
Arguments:
    estimated_elevations - A (P, 3) ndarray containing the x,y,z coordinates of
        each estimated point.

    ground_truth_elevations - An (M, N) ndarray containing the ground truth
        elevations.

    ground_truth_indices - A tuple containing two (P,)-shaped ndarrays with the
        indices (x and y, respectively), of the bins in the ground truth DEM
        corresponding to each estimated point.

Returns:
    None
'''
def show_RMSE(estimated_elevations, ground_truth_elevations, ground_truth_indices):
    absolute_elevation_difference = (
        estimated_elevations - ground_truth_elevations[ground_truth_indices])

    # Compute the root mean squared error (RMSE) for each bin in the ground truth DEM.
    binned_rmse = np.full_like(ground_truth_elevations, np.nan)
    binned_rmse[ground_truth_indices] = absolute_elevation_difference
    binned_rmse = np.sqrt(binned_rmse ** 2)
    plt.imshow(binned_rmse)
    plt.title('RMSE of estimated elevation with respect to ground truth')
    colorbar = plt.colorbar()
    colorbar.set_label('RMSE (meters)', rotation = 270, labelpad = 20)
    plt.axes().get_xaxis().set_visible(False)
    plt.axes().get_yaxis().set_visible(False)
    plt.show()

    # Compute the total RMSE over the entire map.
    total_rmse = np.sqrt(np.sum(absolute_elevation_difference ** 2) / estimated_elevations.shape[0])
    print '    Total RMSE: ', total_rmse

    print '    Estimates variance: ', np.var(estimated_elevations)

def main():
    parser = argparse.ArgumentParser(
        description = 'Plot the reconstructed 3D points.')
    parser.add_argument('true_elevations', type = str,
        help = 'path to a file containing the true 3D topography of the region.' +
               'this file should be an elevation raster, e.g. a digital elevation model')
    parser.add_argument('estimated_elevations', type = str,
        help = 'path to a file containing the 3D points reconstructed by bundle adjustment. ' +
               'this file should be a comma-separated list of x,y,z points, one per line.')
    parser.add_argument('estimated_elevations_projection', type = str,
        help = 'wkt representation of the geographic projection for the xy coordinates ' +
               'of the estimated elevations.')

    args = parser.parse_args()

    # Load datasets
    ground_truth_dataset = gdal.Open(args.true_elevations)
    ground_truth_elevations = ground_truth_dataset.ReadAsArray().astype(np.float)

    estimates = get_projected_estimates(
        np.loadtxt(args.estimated_elevations, delimiter = ','),
        args.estimated_elevations_projection,
        ground_truth_dataset, ground_truth_elevations)
    assert(estimates.shape[1] == 3)

    ground_truth_indices = get_corresponding_ground_truth_points(
        estimates, ground_truth_dataset, ground_truth_elevations)
    assert(len(ground_truth_indices) == 2)
    assert(ground_truth_indices[0].shape == (estimates.shape[0],))
    assert(ground_truth_indices[1].shape == (estimates.shape[0],))

    # Evaluate various estimates against the ground truth.
    print 'Ground truth'
    show_RMSE(
        ground_truth_elevations[ground_truth_indices],
        ground_truth_elevations, ground_truth_indices)

    print 'Flat estimated elevations (null hypothesis)'
    flat_estimated_elevations = np.full_like(
        estimates[:,2], np.average(ground_truth_elevations))
    show_RMSE(
        flat_estimated_elevations, ground_truth_elevations, ground_truth_indices)

    print 'Unscaled estimated elevations'
    show_RMSE(
            estimates[:,2], ground_truth_elevations, ground_truth_indices)

    scaling_matrix, translation_vector = get_estimate_to_ground_truth_affine_map(
        estimates, ground_truth_elevations, ground_truth_dataset, ground_truth_indices)
    affine_estimated_elevations = scaling_matrix.dot(estimates.T).T + translation_vector

    print 'Full affine-fit estimated elevations'
    show_RMSE(
        affine_estimated_elevations, ground_truth_elevations, ground_truth_indices)

    # Export data for rendering in Meshlab and Blender.
    export_points_for_rendering(
        estimates, ground_truth_elevations,
        ground_truth_dataset, ground_truth_indices)


if __name__ == '__main__':
    sys.exit(main())
