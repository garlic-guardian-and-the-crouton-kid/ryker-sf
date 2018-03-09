3D reconstruction of 1938 San Francisco
=======================================

images.txt 
	- This file contains the URLs of GeoTIFFs representing orthorectified
      images taken by Harrison Ryker of San Francisco in 1938, digitized
      by the David Rumsey Map Collection. The line number in images.txt
      indicates the number of the image in the Map Collection.

### Partitioning
The code which partitions the georectified images into overlapping sets
uses [CGAL](https://www.cgal.org/), so you will need to install the dev
libraries for CGAL on your machine before building.

To build and run the partitioning code, run:

```
mkdir -p build
cd build && cmake .. && make
./reconstruct_3d
```
