import numpy as np
import math

import time


def canny(in_array, out_array, width, hight, threashold):

    threashold = threashold ** 2
    start = time.time()
    temp_array = np.zeros(width * hight, dtype=np.uint32)
    # temp_angle = np.zeros(width * hight, dtype=np.int8)
    yt = 2 * width - 1
    yb = -1
    for y in range(width, (hight - 1) * width, width):
        yt += 2
        yb += 2
        dy = [in_array[yt - 1] - in_array[yb - 1], in_array[yt] - in_array[yb]]
        sx = [in_array[yb - 1] + in_array[y] + in_array[yt - 1], in_array[yb] + in_array[y + 1] + in_array[yt]]
        flag = 0
        for x in range(1, width - 1):
            yt += 1
            yb += 1
            dr = in_array[yt] - in_array[yb]
            sr = in_array[yt] + in_array[y + x + 1] + in_array[yb]

            result = ((sr - sx[flag]) ** 2 + (dy[0] + dy[1] + dr) ** 2)

            # xv = sr - sx[flag]
            # yv = dy[0] + dy[1] + dr
            # result = xv ** 2 + yv ** 2

            dy[flag] = dr
            sx[flag] = sr
            flag = 1 - flag

            if result > threashold:
                temp_array[x + y] = result
                # if xv == 0:
                #     temp_angle[x + y] = 90
                # else:
                #     temp_angle[x + y] = math.atan(yv / xv) / math.pi * 18

    for y in range(width, (hight - 1) * width, width):
        for index in range(y + 1, y + width - 1):
            
            if temp_array[index] > temp_array[index - 1] and temp_array[index] > temp_array[index + 1] and temp_array[index] > temp_array[index - width] and temp_array[index] > temp_array[index + width]:
                out_array[index] = 255
            
            """
            if temp_array[index] == 0:
                continue

            if abs(temp_angle[index]) < 45:
                if temp_array[index] > temp_array[index + 1] and temp_array[index] > temp_array[index - 1]:
                    out_array[index] = 255
            else:
                if temp_array[index] > temp_array[index + width] and temp_array[index] > temp_array[index - width]:
                    out_array[index] = 255
            """
            """
            if temp_array[index] == 0:
                continue

            if -60 < temp_angle[index] < -30:
                if temp_array[index] > temp_array[index + width - 1] and temp_array[index] > temp_array[index - width + 1]:
                    out_array[index] = 255
            elif -30 <= temp_angle[index] < 30:
                if temp_array[index] > temp_array[index + 1] and temp_array[index] > temp_array[index - 1]:
                   out_array[index] = 255
            elif 30 <= temp_angle[index] < 60:
                if temp_array[index] > temp_array[index + width + 1] and temp_array[index] > temp_array[index - width - 1]:
                    out_array[index] = 255
            else:
                if temp_array[index] > temp_array[index + width] and temp_array[index] > temp_array[index - width]:
                    out_array[index] = 255
            """
    print 'canny detector spend', time.time() - start, 'seconds'

    """
    DSU_array = np.zeros(width * hight, dtype=np.int8)
    subsets = []

    dist_sup_x = 3
    dist_sup_y = dist_sup_x * width

    for y0 in range(dist_sup_y, (hight - dist_sup_x) * width, dist_sup_y):
        for index0 in range(y0 + dist_sup_x, y0 + width - dist_sup_x, dist_sup_x):
            if out_array[index0] == 255:
                link = []
                for y in range(index - dist_sup_y, index + dist_sup_y + 1, dist_sup_y):
                    for index in range(y - dist_sup_x, y + dist_sup_y + 1):
                        if out_array[index] == 255:
                            link.append(index)
                for point in link:
                    if DSU_array[point] > 0
    """
