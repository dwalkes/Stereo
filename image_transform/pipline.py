import sys, os
import transformImage
import undistort
from subprocess import call
import cv2
import numpy as np

def copyImage(source, dest):
    cv2.imwrite(dest, cv2.imread(source))

def getMask(template, img):
    tmp = cv2.absdiff(template, img)
    mean = np.mean(tmp)
    std = np.std(tmp)
    print mean, std
    mask = img > mean*2.0
    return mask

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print 'USAGE ./pipline <source_dir> <dir to transformed image> <dir for undistorted image> <depth map dir>'
    source_dir = sys.argv[1]
    work_dir = sys.argv[2]
    transform_dir = os.path.join(work_dir, 'imagetransform')
    undistort_dir = os.path.join(work_dir, 'undistort')
    processing_dir = os.path.join(work_dir, 'process')
    depth_name = os.path.join(work_dir, 'depth')
    '''transform_dir = sys.argv[2]
    undistort_dir = sys.argv[3]
    depth_name = sys.argv[4]
    processing_dir = sys.argv[5]'''

    ltemplate = cv2.pyrDown(cv2.imread(os.path.join(source_dir, '00100-left.jpg'), 0))
    rtemplate = cv2.pyrDown(cv2.imread(os.path.join(source_dir, '00100-right.jpg'), 0))
    #ltemplate = cv2.pyrDown(cv2.imread(os.path.join(source_dir, '25100-left.jpg'), 0))
    #rtemplate = cv2.pyrDown(cv2.imread(os.path.join(source_dir, '25100-right.jpg'), 0))

    #for fname in filter(lambda x: x[-8:] == 'left.jpg', os.listdir(source_dir)):
    for fname in os.listdir(source_dir):
        print fname
        if fname[-8:] == 'left.jpg':
            transformImage.main(['', 'H3', os.path.join(source_dir, fname), os.path.join(transform_dir, fname)])
        else:
            copyImage(os.path.join(source_dir, fname), os.path.join(transform_dir, fname))

    for fname in os.listdir(transform_dir):
        print fname
        if fname[-8:] == 'left.jpg':
            undistort.main(['', 'l1try', os.path.join(transform_dir, fname), os.path.join(undistort_dir, fname)])
        else:
            undistort.main(['', 'r1try', os.path.join(transform_dir, fname), os.path.join(undistort_dir, fname)])

    for fname in filter(lambda x: x[-8:] == 'left.jpg',  os.listdir(undistort_dir)):
        lfname = fname
        fname = fname[:-8]
        rfname = fname + 'right.jpg'
        print fname, depth_name

        limg = cv2.imread(os.path.join(undistort_dir, lfname), 0)
        limg = cv2.pyrDown(limg)
        lmask = getMask(ltemplate, limg)
        #limg = limg*lmask
        cv2.imwrite(os.path.join(processing_dir, lfname), limg)

        rimg = cv2.imread(os.path.join(undistort_dir, rfname), 0)
        rimg = cv2.pyrDown(rimg)
        rmask = getMask(rtemplate, rimg)
        #rimg = rimg*rmask
        cv2.imwrite(os.path.join(processing_dir, rfname), rimg)
        
        call(['./depthmap', 'depthparams.yaml', os.path.join(processing_dir, lfname), os.path.join(processing_dir, rfname), os.path.join(depth_name, fname + '.jpg')])

        #cv2.imwrite(os.path.join(depth_name, fname+'.jpg'),  cv2.imread(os.path.join(depth_name, fname+'.jpg'), 0) * lmask)

        #call(['./depthmap', 'depthparams.yaml', os.path.join(transform_dir, lfname), os.path.join(source_dir, rfname), os.path.join(depth_name, fname + '.jpg')])
