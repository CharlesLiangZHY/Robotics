import math
import random

import time


def ransac_cicle(in_array, width, height):

    start = time.time()

    radius_dif = 2

    circle_x = 0
    circle_y = 0
    circle_radius = 0

    edge = []
    for index, pix in enumerate(in_array):
        if pix == 255:
            edge.append([index % width, index // width])

    flag = 0
    min = None
    max = None
    while True:

        sample = random.sample(edge, 3)

        X1 = sample[0][0] - sample[1][0]
        Y1 = sample[0][1] - sample[1][1]
        X2 = sample[0][0] - sample[2][0]
        Y2 = sample[0][1] - sample[2][1]

        D = 2 * (X1 * Y2 - X2 * Y1)
        if D == 0:
            continue

        P0 = sample[0][0] ** 2 + sample[0][1] ** 2
        P1 = P0 - sample[1][0] ** 2 - sample[1][1] ** 2
        P2 = P0 - sample[2][0] ** 2 - sample[2][1] ** 2

        center_x = int(round((P1 * Y2 - P2 * Y1) / D))
        center_y = int(round((X1 * P2 - X2 * P1) / D))
        radius = int(round(((center_x - sample[0][0]) ** 2 + (center_y - sample[0][1]) ** 2) ** 0.5))

        if radius > 175 or radius < 5:
            continue

        inliner = []

        max_dif = radius + radius_dif
        for pix in edge:
            #if pix[1] <= center_y - max_dif:
            #    continue
            #if pix[1] >= center_y + max_dif:
            #    break
            if abs(((pix[0] - center_x) ** 2 + (pix[1] - center_y) ** 2) ** 0.5 - radius) <= radius_dif:
                inliner.append(pix)

        #if min == None or min > len(edge) / len(inliner):
        #    min = len(edge) / len(inliner)

        #if len(edge) / len(inliner) < 10 and (max == None or max < len(inliner) / radius):
        #    max = len(inliner) / radius

        if flag > 2000:
            radius = 0
            break 
        elif len(edge) / len(inliner) < 15 and len(inliner) / float(radius) > 0.8:
            for point in inliner:
                in_array[point[0] + point[1] * width] = 2
        #    print 'find circle', flag
            break
        else:
            flag += 1

    #print max , min

    print 'ransac circle detector spend', time.time() - start, 'seconds'

    return center_x, center_y, radius










