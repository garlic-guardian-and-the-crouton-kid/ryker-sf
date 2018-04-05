3D reconstruction of 1938 San Francisco
=======================================

This repository contains a pipeline for reconstructing 3D topography from a set
of partially-overlapping images taken by uncalibrated cameras. We demonstrate
this system on [a set of aerial images of San Francisco taken by Harrison Ryker
in 1938][aerial photographs], preserved by the San Francisco Public Library, and
digitized by the [David Rumsey Map Center][].

![map showing the error in our reconstruction][error_map]

The map above shows the [RMSE][] of our reconstruction, compared against a [2013
digital elevation map][] (DEM) with 1/3 arc-second resolution (approximately 10
meters). Our point cloud reconstruction is binned at the resolution of the DEM.
More information about these DEMs is available [here][USGS DEMs].

Total RMSE for our reconstruction, compared against the 2013 DEM is 35.42
meters.

Some renderings of our reconstruction:

![estimates_with_ground_truth_overhead.png][]

Estimates (silver) and ground truth (gold). North is up.

![estimates_with_ground_truth_side.png][]

Estimates (silver) and ground truth (gold). Profile view.

![estimates_with_ground_truth_tilted.png][]

Estimates (silver) and ground truth (gold). Northeast is up.

![estimates_with_ground_truth_overhead_2.png][]

Estimates (silver) and ground truth (gold). North is up.

![ground_truth_overhead.png][]

Ground truth. North is up.

In all renderings above, the ground truth dataset (DEM) is in gold, while our
estimates are in silver. Note that the ground truth dataset is complete; we
have cropped it here to show only those latlngs for which we have elevation
estimates.

See [Rendering](#rendering) below for the details of how this rendering was created.

Here's how our reconstruction stacks up against the ground truth:

| Dataset                         |  [RMSE][] (meters) | Variance
|---------------------------------|--------------------|---------
| Ground truth elevation          | 0                  | 1462.82
| Unscaled (raw) estimates        | 63.66              | 0.000024
| Flat estimate (null hypothesis) | 41.55              | 0
| Affine-aligned estimates        | 35.42              | 208.35

The RMSE for the ground truth elevation is 0 by definition. The unscaled (raw)
estimates are the raw values output from the reconstruction pipeline. These raw
values are fit to the ground truth elevations using an affine transformation to
obtain the "affine-aligned estimates." The "flat estimate" represents the null
hypothesis for reconstruction -- a flat horizontal plane located at the average
elevation of the ground truth dataset.

The table above shows that the aligned estimates are substantially better than
the raw estimates (as is to be expected, since structure-from-motion methods
leave an affine ambiguity in the 3D reconstruction). The table also shows that
our aligned estimates perform only marginally better than the null hypothesis.
The variance calculations reveal that the reconstructed topography is too flat.

Further work should investigate why the reconstruction is so flat. The
elevations are initialized to the same height at the beginning of bundle
adjustment, so it is possible that bundle adjustment is simply not running for
enough iterations. Further tuning of bundle adjustment could yield gains in
accuracy and articulation in the reconstructed surface.

### Running

To run the pipeline, build the code (see below for instructions), and then run:

```
find $PWD/$IMAGES -type f | xargs reconstruct_3d
```

where $IMAGES is the (local) path to the directory where you have downloaded the
images.

### Development
The code which partitions the georectified images into overlapping sets
uses [CGAL](https://www.cgal.org/), so you will need to install the dev
libraries for CGAL on your machine before building. On Ubuntu, you can do
this using apt-get. You will also need to install and build [SBA][] and its
dependencies, which you can do as follows:

```
curl -H "User-Agent: Mozilla/5.0" \
    http://users.ics.forth.gr/~lourakis/sba/sba-1.6.tgz > sba-1.6.tgz
tar -xvzf sba-1.6.tgz -C third_party
cd third_party/sba-1.6
cmake .
make sba
```

It is ok if `cmake .` and `make sba` output warnings. Make sure to run
`make sba` and not `make`, as the eucsbademo target is broken. To link
sba, you will need lapack and BLAS, which you can install on ubuntu via
`apt-get`.

To build and run the C++ code, run:

```
export SBAROOT=$PWD/third_party/sba-1.6
mkdir -p build
cd build && cmake .. && make
find /path/to/src/images -type f | xargs ./reconstruct_3d
```

Before submitting your code, you should run the Google C++ linter to ensure
consistent style (you can install with `pip install cpplint`).

```
cpplint --filter=-build/include_subdir include/*.h src/*.cc
```

If there are lint errors, clang-format can probably fix many of them:

```
find src include -type f | xargs clang-format -style=google -i
```

### Downloading images
Use the script in `scripts/download_images.py`:

```
mkdir images/full_resolution -p
python scripts/download_images.py images.txt images/full_resolution
```

The images.txt file contains the URLs of GeoTIFFs representing orthorectified
images taken by Harrison Ryker of San Francisco in 1938, digitized by the David
Rumsey Map Collection. The line number in images.txt indicates the number of the
image in the Map Collection.

NOTE: All of the images are in their original state, as georeferenced by the
David Rumsey Map Collection, except for image 61, which had only 2 control points.
I added several more ground control points in order to orthorectify this image.
  - Images 143 and 148 are missing because they lack sufficient features to identify
    point matches, between the image and the satellite layer, even by hand.
  - Bad images: 45, 101

### Preprocessing images
The GeoTIFF images available on Georeferencer ([example][]) are extremely large
(e.g. 11347x13517). Resizing the images is necessary to make the pipeline
tractable. You can use [GDAL](http://www.gdal.org/) to resize the downloaded
GEOTIFFs. For example, to resize an image called input.tif to be 512 pixels
wide, maintaining the aspect ratio of the image, and write it to a new file
output.tif, run:

```
gdalwarp -of GTiff -ts 512 0 input.tif output.tif
```

See [this answer][gdalwarp image resize] on StackOverflow for more details.

### Evaluation

We evaluate the quality of our reconstruction against a 2013 DEM of San
Francisco. To run this evaluation yourself, you will first need to download the 
DEM yourself from the USGS website.

```
curl https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/13/ArcGrid/n38w123.zip > sf_dem.zip
unzip sf_dem.zip -d sf_dem
```

This DEM covers the entire San Francisco Bay Area, so we crop it using GDAL to
cover only San Francisco:

```
gdalwarp sf_dem/grdn38w123_13/w001001x.adf -te \
    -122.533879065 37.6915998476 \
    -122.343652693 37.8195619919 \
    SanFranciscoCropped.adf
```

You can then run evalution with:

```
SRC_IMAGE_PROJECTION=$(gdalinfo /path/to/a/src/image.tif -json -proj4 | jq -r '.coordinateSystem.proj4')
python scripts/evaluate_reconstruction.py SanFranciscoCropped.adf \
    /path/to/adjusted_points.csv $SRC_IMAGE_PROJECTION
```

Note that this assumes you have already run the reconstruction pipeline;
adjusted_points.csv is one of the files that the reconstruction pipeline
outputs. Each line in this file contains the x,y,z coordinates of a
reconstructed point (x and y coordinates are in the coordinate reference
system of the source images).

Note that if you are running this script in a virtual environment (e.g.
virtualenv), you may have trouble installing GDAL. The following should work:

```
pip download GDAL
tar -zxvf GDAL-X.Y.Z.tar.gz
cd GDAL-1.11.2
python setup.py build_ext --include-dirs=/usr/include/gdal/
python setup.py build
python setup.py install
```

where X.Y.Z should match the version of GDAL you already have installed on
your system (check `gdalinfo --version`). See
[Python GDAL package missing header file][], [Installing GDAL in a Python
virtual environment][], [Installing GDAL into a python virtualenv][].


### Rendering

The renderings of the reconstructed ground surface of San Francisco were created
in [Meshlab][] and [Blender][]. To reproduce:
  1. Import the reconstructed points into Meshlab as a .asc file (plain list of
     x,y,z points)
  2. Reconstruct the meshed surface. Remeshing, Simplification, and
     Reconstrution > Surface Reconstruction: Ball-pivoting (again, use the
     default settings). Note that ball-pivoting works better for our partial
     reconstruction than Poisson reconstruction because it preserves the
     original points along with the holes which indicate missing data. Note
     that the ball-pivoting algorithm requires that data on the x, y, and z
     axes have similar ranges.
  3. Export mesh to STL.
  4. Open the STL file in Blender and experiment with different camera angles
     to find angles which convey a qualitative sense of the topography of the
     reconstruction. Use the numeric keys to rotate the camera by increments of
     15 degrees about the x, y, and z axes. Color the dataset different colors
     by selecting each dataset in turn and creating a new material for each
     selection. Change the background color, if desired, by changing the theme
     background color in File > User Preferences > Themes > 3D editor. Take
     screenshots.
  5. Done!

[aerial photographs]: https://www.davidrumsey.com/luna/servlet/detail/RUMSEY~8~1~217219~5504219:Composite--1-164-San-Francisco-Aeri?sort=Pub_List_No_InitialSort%2CPub_Date%2CPub_List_No%2CSeries_No&qvq=q:ryker;sort:Pub_List_No_InitialSort%2CPub_Date%2CPub_List_No%2CSeries_No;lc:RUMSEY~8~1&mi=0&trs=166
[David Rumsey Map Center]: https://www.davidrumsey.com/
[example]: https://davidrumsey.georeferencer.com/maps/280343924889/
[gdalwarp image resize]: https://gis.stackexchange.com/questions/111523/how-to-correctly-resize-raster-gis-images-to-a-given-px-width
[SBA]: http://users.ics.forth.gr/~lourakis/sba/
[error_map]: https://github.com/garlic-guardian-and-the-crouton-kid/ryker-sf/blob/master/results/error_map.png
[RMSE]: https://en.wikipedia.org/wiki/Root-mean-square_deviation
[2013 digital elevation map]: https://www.sciencebase.gov/catalog/item/581d224ee4b08da350d547ca
[USGS DEMs]: https://catalog.data.gov/dataset/usgs-national-elevation-dataset-ned-1-meter-downloadable-data-collection-from-the-national-map-
[Blender]: https://www.blender.org/
[Meshlab]: http://www.meshlab.net/
[Python GDAL package missing header file]: https://gis.stackexchange.com/questions/28966/python-gdal-package-missing-header-file-when-installing-via-pip
[Installing GDAL in a Python virtual environment]: https://gist.github.com/cspanring/5680334
[Installing GDAL into a python virtualenv]: https://gis.stackexchange.com/questions/6360/installing-geos-proj-gdal-ogr-into-a-python-virtualenv-on-mac-os-x

[estimates_with_ground_truth_overhead.png]: https://github.com/garlic-guardian-and-the-crouton-kid/ryker-sf/blob/master/results/estimates_with_ground_truth_overhead.png
[estimates_with_ground_truth_tilted.png]: https://github.com/garlic-guardian-and-the-crouton-kid/ryker-sf/blob/master/results/estimates_with_ground_truth_tilted.png
[estimates_with_ground_truth_side.png]: https://github.com/garlic-guardian-and-the-crouton-kid/ryker-sf/blob/master/results/estimates_with_ground_truth_side.png
[ground_truth_overhead.png]: https://github.com/garlic-guardian-and-the-crouton-kid/ryker-sf/blob/master/results/ground_truth_overhead.png
[estimates_with_ground_truth_overhead_2.png]: https://github.com/garlic-guardian-and-the-crouton-kid/ryker-sf/blob/master/results/estimates_with_ground_truth_overhead_2.png


