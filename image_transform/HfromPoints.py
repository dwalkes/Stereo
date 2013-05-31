import cv2, sys, numpy as np

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print 'USAGE python HfromPoints <ransac value>  <points_file> <outfile>'
        exit()
    m1, m2 = [], []
    with open(sys.argv[2]) as f:
        for string in f:
            params = map(float, string.split())
            m1.append(params[:2])
            m2.append(params[2:])
    m1 = np.array(m1)
    m2 = np.array(m2)
    H, status = cv2.findHomography(m1, m2, cv2.RANSAC, float(sys.argv[1]))
    with open(sys.argv[3], 'w') as f: np.save(f, H)
