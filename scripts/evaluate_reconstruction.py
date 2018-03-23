from osgeo import gdal
import numpy as np
import sys
from pyproj import Proj
import matplotlib.pyplot as plt

def get_geo_transform(gdal_data):
    T = gdal_data.GetGeoTransform()
    return np.vstack((T[0:3], T[3:6]))

point_cloud_projection = pyproj.Proj("+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs")
reconstructed_points_filename = ""

def normalize(M):
    scaledM = M / (np.max(M) - np.min(M))
    mean = np.average(scaledM)
    return scaledM - mean

def homogenize():
    pass

def dehomogenize():
    pass

def main():
    elevations_dataset = gdal.Open("/home/justin/p/rykers-sf/sf_dem/grdn38w123_13/w001001x.adf")
    elevations_projection = pyproj.Proj(gdal_data.GetProjection())
    elevations_geo_transform = get_geo_transform(elevations_dataset)

    elevations = elevations_dataset.ReadAsArray().astype(np.float)

    projected_elevations = np.apply_along_axis(
        partial(elevation_to_pointcloud_2d, geo_transform),
        0, 
        np.indices(elevations.shape))
    assert(projected_elevations.shape == (2,) + elevations.shape)

    elevation_points = np.concatenate((elevations, projected_elevations), 0)
        .reshape(3, elevations.shape[0] * elevations.shape[1])
    reconstructed_points = np.loadtxt(reconstructed_points_filename)
    assert(reconstructed_points.shape[0] == 3)
    print 'reconstructed_points.shape', reconstructed_points.shape

    def reconstruction_to_DEM(p):
        return pyproj.transform(
            point_cloud_projection,
            elevations_projection,
            p[0], p[1], p[2])

    # TODO: Build an array of elevation points which correspond to the
    # reconstructed 3D points.
    reconstructed_points_projected = np.apply_along_axis(
        reconstruction_to_DEM,
        0,
        reconstructed_points)

    reconstruction_DEM_indices = (
        np.digitize(
                reconstructed_points_projected[0],
                np.arange(elevations.shape[0])),
        np.digitize(
                reconstructed_points_projected[1],
                np.arange(elevations.shape[1])))
    
    reconstructed_point_measurements = np.empty(
        3 * reconstructed_points.shape[0], reconstructed_points.shape[1])
    reconstructed_point_measurements[0::3] = reconstructed_points
    reconstructed_point_measurements[1::3] = reconstructed_points
    reconstructed_point_measurements[2::3] = reconstructed_points

    elevation_measurements = np.vstack((
        dehomogenize(geo_transform * homogenize(np.indices(elevations.shape)))
        elevations[reconstruction_DEM_indices]
    ))

    # Map which transforms the reconstructed points into the points in the
    # digital elevation model.
    affine_map = np.linalg.lstsq(
        reconstructed_point_measurements,
        elevations[reconstruction_DEM_indices].flatten())

    transformed_reconstructed_points =
        affine_map.dot(homogenize(reconstructed_points))

    # Bin the points and get their elevation. Should be an N x M array with the
    # values indicating the average elevation.
    reconstruction_indices = (
        np.histogram(transformed_reconstructed_points[0,:], elevations.shape[0]),
        np.histogram(transformed_reconstructed_points[1,:], elevations.shape[1]))
    aligned_reconstruction = transformed_reconstructed_points[reconstruction_indices]

    plt.imshow(normalize(aligned_reconstruction - elevation))

if __name__ == '__main__':
    sys.exit(main())
