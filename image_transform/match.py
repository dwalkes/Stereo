import numpy as np
import cv2
from functools import partial
import numpy as np

help_message = '''SURF image scene match 
press 'X' for make photo
'''

FLANN_INDEX_KDTREE = 1  # bug: flann enums are missing
flann_params = dict(algorithm = FLANN_INDEX_KDTREE,
                    trees = 4)

index = 0

def match_flann(desc1, desc2, r_threshold = 0.6):
    flann = cv2.flann_Index(desc2, flann_params)
    idx2, dist = flann.knnSearch(desc1, 2, params = {}) # bug: need to provide empty dict
    mask = dist[:,0] / dist[:,1] < r_threshold
    idx1 = np.arange(len(desc1))
    pairs = np.int32( zip(idx1, idx2[:,0]) )
    return pairs[mask]


def draw_match(img1, img2, p1, p2, status = None, H = None, index=index):
    h1, w1 = 0, 0 #img1.shape[:2]
    h2, w2 = 0, 0 #img2.shape[:2]
    tmp_img = img1/2 + img2/2
    vis = cv2.cvtColor(tmp_img.copy(), cv2.COLOR_GRAY2BGR)
    #vis = cv2.cvtColor(img1.copy(), cv2.COLOR_GRAY2BGR)
    #for x, y in np.int32(p2):
    #    cv2.circle(vis, (x, y), 2, (255, 0, 0), -1)
    #for x, y in np.int32(p1):
    #    cv2.circle(vis, (x, y), 2, (0, 0, 255), -1)

    '''
    vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
    vis[:h1, :w1] = img1
    vis[:h2, w1:w1+w2] = img2
    vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    
    if H is not None:
        corners = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]])
        corners = np.int32( cv2.perspectiveTransform(corners.reshape(1, -1, 2), H).reshape(-1, 2) + (w1, 0) )
        #cv2.polylines(vis, [corners], True, (255, 255, 255))
    '''
    if status is None:
        status = np.ones(len(p1), np.bool_)
    green = (0, 255, 0)
    red = (0, 0, 255)
    res_error = 0
    count = 0

    #tr_img = cv2.perspectiveTransform(tmp_img, H)
    tr1_img = cv2.warpPerspective(img1, H, (tmp_img.shape[1], tmp_img.shape[0]))
    cv2.imshow('trans1', tr1_img)
    cv2.imwrite('match_dump/%d.jpg' % index, tr1_img)
    #cv2.imshow('trans2', tr2_img)
    print H
    for (x1, y1), (x2, y2), inlier in zip(np.int32(p1), np.int32(p2), status):
        col = [red, green][inlier]
        if inlier:
            cv2.line(vis, (x1, y1), (x2+w1, y2), col)
            res_error += abs(x1 - x2) + abs(y1 - y2)
            count += 1
            cv2.circle(vis, (x1, y1), 2, col, -1)
            cv2.circle(vis, (x2+w1, y2), 2, col, -1)
        else:
            r = 2
            thickness = 3
            cv2.line(vis, (x1-r, y1-r), (x1+r, y1+r), col, thickness)
            cv2.line(vis, (x1-r, y1+r), (x1+r, y1-r), col, thickness)
            cv2.line(vis, (x2+w1-r, y2-r), (x2+w1+r, y2+r), col, thickness)
            cv2.line(vis, (x2+w1-r, y2+r), (x2+w1+r, y2-r), col, thickness)
    print 'error', res_error/float(count) if count > 0 else 0.000001
    return vis

def params_from_image(img, surf=None):
    if surf == None:surf = cv2.SURF(1000)
    #kp, desc = surf.detect(img, None, False)
    kp, desc = surf.detectAndCompute(img, None)
    desc.shape = (-1, surf.descriptorSize())
    return {'img':img, 'kp':kp, 'desc':desc}

def template_match(params_orig, params_template, rans, r_threshold):
    img1, kp1, desc1 = params_orig['img'], params_orig['kp'], params_orig['desc']
    img2, kp2, desc2 = params_template['img'], params_template['kp'], params_template['desc']

    #r_threshold = 0.95
    m = match_flann(desc1, desc2, r_threshold)
    dtmp = {}
    for i, j in m: dtmp[j] = i
    m = [[i, j] for j, i in dtmp.iteritems()]
    matched_p1 = np.array([kp1[i].pt for i, j in m])
    matched_p2 = np.array([kp2[j].pt for i, j in m])

    try:
        #H, status = cv2.findHomography(matched_p1, matched_p2, cv2.RANSAC, 100.0)
        H, status = cv2.findHomography(matched_p1, matched_p2, cv2.RANSAC, rans)
    except:
        H = None
        status = None
    vis_flann = draw_match(img1, img2, matched_p1, matched_p2, status, H)
    print H
    return vis_flann, status, matched_p1, matched_p2

if __name__ == '__main__':
    import sys
    print 'usage: match.py template img points image'
    try: fn1, fn2, out_file_name = sys.argv[1:4]
    except:
        fn1 = 'template.jpg'
        fn2 = 'img.jpg'
        print help_message

    template = cv2.imread(fn1)
    frame = cv2.imread(fn2)
    rans = 65.
    r_threshold = 0.9
    while 1:
        #template = None
        surf = cv2.SURF(1000)
        pr1 = params_from_image(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))#img1)
        pr2 = params_from_image(cv2.cvtColor(template, cv2.COLOR_BGR2GRAY))#img2)
        print 'img1 - %d features, img2 - %d features' % (len(pr1['kp']), len(pr2['kp']))
        vis_flann, status, matched_p1, matched_p2 = template_match(pr1, pr2, rans, r_threshold)
        index += 1
        print 'flann match:',
        if status != None:
            print '%d / %d  inliers/matched' % (np.sum(status), len(status))
        cv2.imshow('find_obj SURF flann', vis_flann)
        key = cv2.waitKey() % 0x100
        print key, 'RANSAC', rans, 'r_threshold', r_threshold
        if key == 81: rans -= 10
        if key == 83: rans += 10
        if key == 84: r_threshold += 0.1
        if key == 82: r_threshold -= 0.1
        if key == 27: exit()
        if key == 32:
            with open(out_file_name, 'w') as f:
                for i in xrange(len(matched_p1)):
                    f.write('%f %f %f %f\n' % (matched_p1[i][0], matched_p1[i][1], matched_p2[i][0], matched_p2[i][1]))


