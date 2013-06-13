import cv2, numpy as np, sys, os

def main(argv):
    r1, r2 = [], []
    with open('points2') as f:
        for s in f:
            x1, y1, x2, y2 = map(float, s.split())
            r1.append([x1, y1])
            r2.append([x2, y2])
    p1, p2 = np.array(r1), np.array(r2)
    ret, mask = cv2.findFundamentalMat(p1, p2)
    print ret


if __name__ == '__main__':
    main(sys.argv)
