import argparse
import math
import os
import urllib2
import sys

'''
Script for downloading images.

This script checks whether the file exists before it attempts to download the
image. If there is an error downloading an image, the script will report the
error and attempt to continue with the next image.
'''
def main():
    parser = argparse.ArgumentParser(
        description = 'Download images from URLs specified in a file')
    parser.add_argument('image_list_filename', type = str,
        help = 'name of a file containing a list of URLs, one on each line')
    parser.add_argument('image_download_dir', type = str,
        help = 'path to the directory where images should be downloaded')
    parser.add_argument('--start_from', type = int, default = 1,
        metavar = 'line_num',
        help = 'the 1-based line number of the first image to download')

    args = parser.parse_args()

    with open(args.image_list_filename, 'r') as image_list:
        num_images = sum(1 for line in open(args.image_list_filename))
        max_digits = int(math.log10(num_images + 1)) + 1
        for i, image_url in enumerate(image_list):
            line_num = i + 1

            if line_num < args.start_from:
                continue

            # Add 1 because the index passed by enumerate() is 0-based, but the
            # indices of images in the David Rumsey Map Collection are 1-based.
            image_filename = os.path.join(
                args.image_download_dir, str(line_num).zfill(max_digits) + '.tif')
            if os.path.exists(image_filename) and os.path.isfile(image_filename):
                print 'Skipping ' + image_filename
                continue

            with open(image_filename, 'wb') as image:
                try:
                    request = urllib2.Request(image_url, headers = {
                            # Hack to persuade the server to send the images.
                            # Without this, the server blocks downloads with
                            # 403 Forbidden.
                            'User-Agent': 'Mozilla/5.0'
                        })
                    image.write(urllib2.urlopen(request).read())
                    print 'Downloaded ' + image_filename
                except (urllib2.HTTPError, urllib2.URLError) as err:
                    print 'Error downloading ' + image_filename, err
                    os.remove(image_filename)
                    continue
    return 0

if __name__ == '__main__':
    sys.exit(main())
