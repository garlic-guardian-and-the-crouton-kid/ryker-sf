3D reconstruction of 1938 San Francisco
=======================================

images.txt 
	- This file contains the URLs of GeoTIFFs representing orthorectified
      images taken by Harrison Ryker of San Francisco in 1938, digitized
      by the David Rumsey Map Collection. The line number in images.txt
      indicates the number of the image in the Map Collection.

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
