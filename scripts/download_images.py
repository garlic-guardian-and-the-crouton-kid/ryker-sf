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
    if len(sys.argv) < 3:
        print "Usage: download_images.py <image_urls_file> <image_download_dir>"
        return 1

    # The path for a file containing a list of image URLs, one on each line.
    image_list_filename = sys.argv[1]
    # The path for a directory where the downloaded images will be placed.
    image_download_dir = sys.argv[2]

    with open(image_list_filename, 'r') as image_list:
        num_images = sum(1 for line in open(image_list_filename))
        max_digits = int(math.log10(num_images)) + 1
        for i, image_url in enumerate(image_list):
            image_filename = os.path.join(
                image_download_dir, str(i).zfill(max_digits) + '.tif')
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
