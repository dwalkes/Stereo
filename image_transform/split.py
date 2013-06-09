import sys, os, cv2

if __name__ == '__main__':
    files = os.listdir('.')
    for fname in files:
        if fname[-4:] != '.jpg': continue
        img = cv2.imread(fname)
        print img.shape
        img1 = img[:, :img.shape[1]/2, :]
        img2 = img[:, img.shape[1]/2:, :]
        print img1.shape, img2.shape
        cv2.imwrite(fname[:-4]+'-left.jpg', img1)
        cv2.imwrite(fname[:-4]+'-right.jpg', img2)
