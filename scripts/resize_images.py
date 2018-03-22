import argparse
import os
import sys

'''
Resize a collection of GeoTIFFs.

This script uses GDAL to resize the GeoTIFFs while preserving their geographic
metadata.
'''
def main():
    parser = argparse.ArgumentParser(
        description = 'Resize a collection of GeoTIFFs')
    parser.add_argument('src_dir', type = str,
        help = 'path to a directory containing the source images.')
    parser.add_argument('dest_dir', type = str,
        help = 'path to the directory where the resized images will be placed')
    parser.add_argument('--height', type = int, default = 0,
        help = 'the height of the resized images; see gdalwarp -ts for usage')
    parser.add_argument('--width', type = int, default = 0,
        help = 'the width of the resized images; see gdalwarp -ts for usage')

    args = parser.parse_args()

    if not (args.height or args.width):
        parser.error('Must provide a width or a height to resize images')
        return 1

    src_image_filenames = [
        f
        for f in os.listdir(args.src_dir) 
        if os.path.isfile(os.path.join(args.src_dir, f))]

    if not os.path.exists(args.dest_dir):
        os.makedirs(args.dest_dir)

    for image_filename in src_image_filenames:
        src_image = os.path.join(args.src_dir, image_filename)
        resized_image = os.path.join(args.dest_dir, image_filename)

        if os.path.exists(resized_image) and os.path.isfile(resized_image):
            print 'Skipping ' + image_filename
            continue
        # Note that this requires that you have GDAL installed on your system.
        os.system(
            'gdalwarp -of GTiff -ts {width} {height} {src_image} {dest_image}'
            .format(
                src_image = src_image,
                dest_image = resized_image,
                height = args.height,
                width = args.width
            ))

    return 0

if __name__ == '__main__':
    sys.exit(main())
