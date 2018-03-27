import argparse
from osgeo import gdal
import numpy as np
import sys
import math
import osr
import pyproj
import matplotlib.pyplot as plt
from functools import partial
from scipy import spatial

def get_geo_transform(gdal_dataset):
    T = gdal_dataset.GetGeoTransform()
    return np.vstack((T[0:3], T[3:6]))

def normalize(M):
    scaledM = M / (np.max(M) - np.min(M))
    mean = np.average(scaledM)
    return scaledM - mean

'''
Arguments:
    ps - An (N, K) dimensional matrix representing N K-dimensinoal Euclidean
    vectors
Returns:
    An (N, K+1) dimensional matrix in which each K-dimensional vector has been
    augmented with a K+1th coordinate, set equal to 1.
'''
def homogenize(ps):
    num_ps = ps.shape[0]
    return np.append(ps, np.ones(num_ps).reshape(num_ps, 1), axis = 1)


'''
Arguments:
    ps - An (N, K) dimensional matrix representing N K-dimensional homogeneous vectors.
Returns:
    An (N, K-1) dimensional matrix in which the remaining K-1 coordinates have been divided
    by the Kth coordinate.
'''
def dehomogenize(ps):
    return ps[:,:-1] / ps[:,-1, np.newaxis]


def get_projection(gdal_dataset):
    proj_wkt = gdal_dataset.GetProjection()
    proj_converter = osr.SpatialReference()
    proj_converter.ImportFromWkt(proj_wkt)
    return pyproj.Proj(proj_converter.ExportToProj4())


def apply_geo_transform(coords, translation, scale):
    return translation + scale * coords

def geo_transform_x(coords, geo_transform):
    return apply_geo_transform(coords, geo_transform[0], geo_transform[1])

def geo_transform_y(coords, geo_transform):
    return apply_geo_transform(coords, geo_transform[3], geo_transform[5])


def get_projected_estimates(estimates, golden_dataset, golden_elevations):
    estimate_projection = pyproj.Proj("+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs")
    golden_projection = get_projection(golden_dataset)
    golden_geo_transform = golden_dataset.GetGeoTransform()

    def project_estimate_to_golden(p):
        return pyproj.transform(
            estimate_projection,
            golden_projection,
            p[0], p[1], p[2])

    # Project the estimates points into the coordinate reference system of
    # the DEM.
    estimates_projected = np.apply_along_axis(
        project_estimate_to_golden, 1, estimates)

    golden_x_bounds = np.sort(
        geo_transform_x(np.array([0, golden_elevations.shape[1]]), golden_geo_transform))
    golden_y_bounds = np.sort(
        geo_transform_y(np.array([0, golden_elevations.shape[0]]), golden_geo_transform))

    print 'golden_x_bounds', golden_x_bounds
    print 'golden_y_bounds', golden_y_bounds

    estimates_projected[:,1] *= -1

    within_x_bounds = np.logical_and(
        golden_x_bounds[0] < estimates_projected[:,0],
        estimates_projected[:,0] < golden_x_bounds[1])
    print 'np.all(within_x_bounds)', np.all(within_x_bounds)
    within_y_bounds = np.logical_and(
        golden_y_bounds[0] < estimates_projected[:,1],
        estimates_projected[:,1] < golden_y_bounds[1])
    # print 'np.all(within_y_bounds)', np.all(within_y_bounds)
    # print within_y_bounds
    print 'estimates_projected[:,1]', estimates_projected[:,1]

    within_bounds = np.logical_and(within_x_bounds, within_y_bounds)

    print 'out-of-bounds estimates', estimates_projected[np.logical_not(within_bounds),:]
    print 'in-bounds estimates', estimates_projected[within_bounds,:]

    # return estimates_projected[within_bounds,:]
    return estimates_projected


def get_estimate_measurements(estimates):
    estimates_point_measurements = np.zeros((
        3 * estimates.shape[0], estimates.shape[1]))
    estimates_point_measurements[0::3] = estimates
    estimates_point_measurements[1::3] = estimates
    estimates_point_measurements[2::3] = estimates
    assert(estimates_point_measurements.shape == (3 * estimates.shape[0], 3))

    return estimates_point_measurements


'''
Returns:
    - A tuple containing the x-indices and the y-indices, respectively, for the
      bins in golden_elevations corresponding to the points in estimates. That
      is, this function returns a tuple (xs, ys) such that (xs[i], ys[i]) is the
      2D index into golden_elevations corresponding to the ith point in
      estimates.
'''
def get_corresponding_golden_points(estimates, golden_dataset, golden_elevations):
    print 'estimates.shape', estimates.shape
    print 'golden_elevations.shape', golden_elevations.shape

    golden_geo_transform = golden_dataset.GetGeoTransform()

    bins = (
        geo_transform_x(np.arange(golden_elevations.shape[1]), golden_geo_transform),
        geo_transform_y(np.arange(golden_elevations.shape[0]), golden_geo_transform))

    # Should have shape (2, num_estimates).
    # Represents the index in the golden array for each estimate.
    # Note that the order is swapped.
    indices = (
        np.digitize(estimates[:,1], bins[1]),
        np.digitize(estimates[:,0], bins[0]))

    return indices


'''
Returns:
    A (3 * P, 1) ndarray. Every three elements in this array will represent a golden point
    which is matched to a point in estimates.
'''
def get_golden_measurements(estimates, golden_dataset, golden_elevations):
    indices = get_corresponding_golden_points(estimates, golden_dataset, golden_elevations)
    print 'max x index', np.max(indices[0])
    print 'min x index', np.min(indices[0])
    print 'max y index', np.max(indices[1])
    print 'min y index', np.min(indices[1])

    print 'indices above xmax', np.count_nonzero(indices[0] > golden_elevations.shape[1])
    print 'indices above ymax', np.count_nonzero(indices[1] > golden_elevations.shape[0])

    print 'indices below xmin', np.count_nonzero(indices[0] < 0)
    print 'indices below ymin', np.count_nonzero(indices[1] < 0)

    np.set_printoptions(threshold = np.nan)

    golden_measurements = np.empty((estimates.shape[0] * 3))
    golden_measurements[0::3] = geo_transform_x(indices[0], golden_geo_transform)
    golden_measurements[1::3] = geo_transform_y(indices[1], golden_geo_transform)
    golden_measurements[2::3] = golden_elevations[indices].flatten()

    """
    print 'golden_measurements.shape', golden_measurements.shape
    print 'golden_elevations.shape', golden_elevations.shape
    print 'np.indices(golden_elevations.shape).shape', np.indices(golden_elevations.shape).shape
    coords = np.indices(golden_elevations.shape).T.reshape(
        2, golden_elevations.shape[0] * golden_elevations.shape[1]).T
    print 'coords.shape', coords.shape
    print 'estimates.shape', estimates.shape

    distances = spatial.distance.cdist(estimates[:,0:2], coords)
    indices = np.argsort(distances, 1)[:, 0]

    print 'geo_transform_x(indices[0], golden_geo_transform).shape', geo_transform_x(indices[0], golden_geo_transform).shape

    golden_measurements = np.empty((estimates.shape[0] * 3))
    golden_measurements[0::3] = geo_transform_x(indices[0], golden_geo_transform)
    golden_measurements[1::3] = geo_transform_y(indices[1], golden_geo_transform)
    golden_measurements[2::3] = estimates[indices].flatten()
    indices = []

    for golden_x in range(golden_elevations.shape[0]):
        for golden_y in range(golden_elevations.shape[1]):
            for estimate in estimates:
                if (geo_transform_x(golden_x, golden_geo_transform) < estimate[0]
                    and estimate[0] < geo_transform_x(golden_x + 1, golden_geo_transform)
                    and geo_transform_y(golden_y, golden_geo_transform) < estimate[1]
                    and estimate[1] < geo_transform_y(golden_y + 1, golden_geo_transform)):
                        indices.append(idx)
    indxs = []

    for estimate in estimates:
        golden_x = math.floor((estimate[0] - golden_geo_transform[0]) / golden_geo_transform[1])
        golden_y = math.floor((estimate[1] - golden_geo_transform[3]) / golden_geo_transform[5])

        indxs.append([golden_x, golden_y])

    indices = np.array(indxs, dtype = np.int64).T

    golden_measurements = np.empty((estimates.shape[0] * 3))
    golden_measurements[0::3] = geo_transform_x(indices[0], golden_geo_transform)
    golden_measurements[1::3] = geo_transform_y(indices[1], golden_geo_transform)
    golden_measurements[2::3] = estimates[indices].flatten()

    print 'len(indices)', len(indices)
    """

    return golden_measurements



def main():
    parser = argparse.ArgumentParser(
        description = 'Plot the reconstructed 3D points.')
    parser.add_argument('reconstructed_points', type = str,
        help = 'path to a file containing the 3D points reconstructed by bundle adjustment.')

    args = parser.parse_args()

    # "/home/justin/p/rykers-sf/sf_dem/grdn38w123_13/w001001x.adf"
    golden_dataset = gdal.Open("/home/justin/p/rykers-sf/SanFranciscoCropped.adf")
    golden_elevations = golden_dataset.ReadAsArray().astype(np.float)

    estimates_filename = args.reconstructed_points
    raw_estimates = np.loadtxt(estimates_filename, delimiter = ',')
    assert(raw_estimates.shape[1] == 3)
    estimates = get_projected_estimates(raw_estimates, golden_dataset, golden_elevations)

    np.savetxt('reconstructed_reprojected.csv', estimates, delimiter = ',')

    print 'max estimated height', np.max(estimates[:,2])
    print 'min estimated height', np.min(estimates[:,2])

    # exaggerated_estimates = estimates
    # exaggerated_estimates[:,2] = 5 * estimates[:,2]

    np.savetxt('exaggerated_estimates.asc', estimates, delimiter = ',')


    # Map which transforms the estimates points into the points in the
    # digital elevation model.
    """
    affine_map_parameters, _, _, _ = np.linalg.lstsq(
        get_estimate_measurements(estimates),
        get_golden_measurements(estimates, golden_dataset, golden_elevations))

    affine_map = affine_map_parameters.reshape(3,3)

    transformed_estimates = affine_map.dot(
        homogenize(estimates))
    """


    # Bin the points and get their elevation. Should be an N x M array with the
    # values indicating the average elevation.
    """
    estimate_to_golden_indices = (
        np.histogram(transformed_estimates[0,:], golden.shape[0]),
        np.histogram(transformed_estimates[1,:], golden.shape[1]))
    aligned_estimate_to_golden = transformed_estimates[estimate_to_golden_indices]

    plt.imshow(normalize(aligned_estimate_to_golden - elevation))
    """

    indices = get_corresponding_golden_points(estimates, golden_dataset, golden_elevations)

    print 'golden_elevations[indices].shape', golden_elevations[indices].shape
    print 'estimates[:,2].shape', estimates[:,2,np.newaxis].shape

    scaling_factor, _, _, _ = np.linalg.lstsq(
        estimates[:,2,np.newaxis], golden_elevations[indices])
    print 'scaling_factor.shape', scaling_factor.shape
    print 'scaling_factor', scaling_factor

    # TODO: Want to show a map of San Francisco with the difference between the average estimate
    # and the ground truth at that point. What to do if there are no estimates at a particular
    # point?
    scaled_estimated_elevations = scaling_factor * estimates[:,2]
    difference_map = np.full_like(golden_elevations, np.nan)
    for i in np.arange(scaled_estimated_elevations.shape[0]): 
        i1, i2 = indices[0][i], indices[1][i]
        if np.isnan(difference_map[i1, i2]):
            difference_map[i1, i2] = 0
        difference_map[i1, i2] += (scaled_estimated_elevations[i] - golden_elevations[i1, i2]) ** 2
    difference_map = np.sqrt(difference_map)

    plt.imshow(difference_map)
    plt.colorbar()
    plt.show()

    total_rmse = 0
    for i in np.arange(scaled_estimated_elevations.shape[0]): 
        i1, i2 = indices[0][i], indices[1][i]
        total_rmse += (scaled_estimated_elevations[i] - golden_elevations[i1, i2]) ** 2
    total_rmse = np.sqrt(total_rmse / scaled_estimated_elevations.shape[0])
    print 'total_rmse', total_rmse

    print 'ground truth variance', np.var(golden_elevations)
    

if __name__ == '__main__':
    sys.exit(main())
