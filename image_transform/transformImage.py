import sys, cv2, numpy as np

def main(argv):
    if len(argv) < 4:
        print 'USAGE transformImage <H_file> <source_image> <dest_image>'
        exit()
    H = np.zeros((3, 3))
    with open(argv[1]) as f:
        i = 0
        for st in f:
            H[i, :] = map(float, st.split())
            i += 1
    img = cv2.imread(argv[2])
    img = cv2.warpPerspective(img, H, (img.shape[1], img.shape[0]))
    cv2.imwrite(argv[3], img)

if __name__ == '__main__':
    main(sys.argv)
