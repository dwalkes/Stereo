import sys, os
import transformImage
import undistort
from subprocess import call
import cv2

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print 'USAGE ./pipline <source_dir> <dir to transformed image> <dir for undistorted image> <depth map dir>'
    dir_name = sys.argv[1]
    tmp_name = sys.argv[2]
    dest_name = sys.argv[3]
    depth_name = sys.argv[4]
    for fname in filter(lambda x: x[-8:] == 'left.jpg', os.listdir(dir_name)):
        print fname
        transformImage.main(['', 'H3', os.path.join(dir_name, fname), os.path.join(tmp_name, fname)])

    for fname in os.listdir(tmp_name):
        print fname
        undistort.main(['', 'l1try', os.path.join(tmp_name, fname), os.path.join(dest_name, fname)])

    for fname in filter(lambda x: x[-9:] == 'right.jpg',  os.listdir(dir_name)):
        print fname
        undistort.main(['', 'r1try', os.path.join(dir_name, fname), os.path.join(dest_name, fname)])



    for fname in filter(lambda x: x[-8:] == 'left.jpg',  os.listdir(dest_name)):
        lfname = fname
        fname = fname[:-8]
        rfname = fname + 'right.jpg'
        print fname, depth_name

        limg = cv2.imread(os.path.join(dest_name, lfname))
        limg = cv2.pyrDown(limg)
        cv2.imwrite('tmp4/' + lfname, limg)
        rimg = cv2.imread(os.path.join(dest_name, rfname))
        rimg = cv2.pyrDown(rimg)
        cv2.imwrite('tmp4/' + rfname, rimg)
        call(['./depthmap', 'depthparams.yaml', 'tmp4/' + lfname, 'tmp4/' + rfname, os.path.join(depth_name, fname + '.jpg')])
        #call(['./depthmap', 'depthparams.yaml', os.path.join(tmp_name, lfname), os.path.join(dir_name, rfname), os.path.join(depth_name, fname + '.jpg')])
