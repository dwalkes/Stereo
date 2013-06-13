import cv2, numpy as np, sys, os

if __name__ == '__main__' :
    source_dir = sys.argv[1]
    target_dir = sys.argv[2]
    for name in filter(lambda f: f[-4:] == '.jpg', os.listdir(source_dir)):
        img  = cv2.imread(os.path.join(source_dir, name))
        imgL = img[:, :img.shape[1]/2]
        imgR = img[:, img.shape[1]/2:]
        cv2.imwrite(os.path.join(target_dir, '%s-left.jpg' % name[:-4]), imgL)
        cv2.imwrite(os.path.join(target_dir, '%s-right.jpg' % name[:-4]), imgR)
        #cv2.imshow('', imgL)
        #cv2.imshow('1', imgR)
        #cv2.waitKey()
