3D reconstruction of 1938 San Francisco
=======================================
### Running

To run the pipeline, build the code (see below for instructions), and the run:

```
find $PWD/$IMAGES -type f | xargs build/reconstruct_3d
```

where $IMAGES is the (local) path to the directory where you have downloaded the
images.

### Development
The code which partitions the georectified images into overlapping sets
uses [CGAL](https://www.cgal.org/), so you will need to install the dev
libraries for CGAL on your machine before building.

To build and run the C++ code, run:

```
mkdir -p build
cd build && cmake .. && make
./reconstruct_3d
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


[example]: https://davidrumsey.georeferencer.com/maps/280343924889/
[gdalwarp image resize]: https://gis.stackexchange.com/questions/111523/how-to-correctly-resize-raster-gis-images-to-a-given-px-width
