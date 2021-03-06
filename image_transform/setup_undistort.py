import sys, cv2, numpy as np
from undistort import readParams

def changeifx(x): mat[0, 0] = (x-100)/0.1; recalc()
def changeify(x): mat[1, 1] = (x-100)/0.1; recalc()
def changeicx(x): mat[0, 2] = x; recalc()
def changeicy(x): mat[1, 2] = x; recalc()
def changeik1(x): params[0, 0] = (x-100)*0.05; recalc()
def changeik2(x): params[0, 1] = (x-100)*0.05; recalc()
def changeik3(x): params[0, 4] = (x-100)*0.05; recalc()
def changeip1(x): params[0, 2] = (x-100)*0.005; recalc()
def changeip2(x): params[0, 3] = (x-100)*0.005; recalc()

def recalc():
    print mat
    print params
    res = cv2.undistort(img, mat, params)
    cv2.imshow('res', res)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print 'USAGE <image> <out>\nSPACE for save'
    img = cv2.imread(sys.argv[1], 0)
    mat = np.zeros((3, 3), dtype=np.float)
    mat[2, 2] = 1
    mat[0, 0] = 1
    mat[1, 1] = 1
    mat[0, 2] = (img.shape[1]-1)*0.5#1
    mat[1, 2] = (img.shape[0]-1)*0.5
    params = np.array([0., 0., 0., 0., 0., 0., 0., 0.])
    params = params.reshape((1, 8))
    if len(sys.argv) > 3:
        mat, params = readParams(sys.argv[3])
    key = 0
    recalc()
    cv2.imshow('img', img)
    cv2.createTrackbar('fx', 'img', int(mat[1, 1]*0.1 + 100), 200, changeifx)
    cv2.createTrackbar('fy', 'img', int(mat[0, 0]*0.1 + 100), 200, changeify)
    cv2.createTrackbar('cx', 'img', int(mat[0, 2]), 700, changeicx)
    cv2.createTrackbar('cy', 'img', int(mat[1, 2]), 700, changeicy)
    cv2.createTrackbar('k1', 'res', int(params[0, 0]/0.05 + 100), 200, changeik1)
    cv2.createTrackbar('k2', 'res', int(params[0, 1]/0.05 + 100), 200, changeik2)
    cv2.createTrackbar('p1', 'res', int(params[0, 2]/0.05 + 100), 200, changeip1)
    cv2.createTrackbar('p2', 'res', int(params[0, 3]/0.05 + 100), 200, changeip2)
    cv2.createTrackbar('k3', 'res', int(params[0, 4]/0.05 + 100), 200, changeik3)
    while key != 27:
        print key
        key = cv2.waitKey() % 0x100
        if key == 32:
            with open(sys.argv[2], 'w') as f:
                for i in xrange(3):
                    f.write('%0.10f %0.10f %0.10f\n' % (mat[i, 0], mat[i, 1], mat[i, 2]))
                for i in xrange(params.shape[1]):
                    f.write('%0.10f ' % params[0, i])

