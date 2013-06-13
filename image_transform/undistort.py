import sys, cv2, numpy as np

def readParams(name):
    mat = np.zeros((3, 3), dtype=np.float)
    params = None
    with open(name) as f:
        i = 0
        strings = f.readlines()
        for st in strings[:3]:
            mat[i, :] = map(float, st.split())
            i += 1
        params = np.array(map(float, strings[3].split())).reshape((1, len(strings[3].split())))
    return mat, params

def main(argv):
    if len(argv) < 4:
        print 'USAGE <params file> <source> <out>'
        exit()
    mat, params = readParams(argv[1])
    img = cv2.imread(argv[2])
    res = cv2.undistort(img, mat, params)
    cv2.imwrite(argv[3], res)

if __name__ == '__main__':
    main(sys)
