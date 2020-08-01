import math
import numpy as np

import time

def hough_square(in_array, width, hight):

    density_r = 200
    density_d = 300
    line_ratio = 30
    a_threas = 2

    start = time.time()

    square_x = 0
    square_y = 0
    square_radius = 0
    square_angle = 0

    edge = []
    for index, pixle in enumerate(in_array):
        if pixle == 255 or pixle == 2:
            x = index % width
            y = index // width
            edge.append([x, y])

    line_inf = len(edge) / line_ratio
    r_sup = ((width - 1) ** 2 + (hight - 1) ** 2) ** 0.5
    r_step = r_sup / density_r
    d_step = math.pi / density_d
    hough = np.empty((density_d, density_r * 2), dtype = object)
    for r in range(density_r * 2):
        for d in range(density_d):
            hough[d][r] = []

    pi_2 = math.pi / 2
    for pixle in edge:
        r = (pixle[0] ** 2 + pixle[1] ** 2) ** 0.5
        if pixle[0] == 0:
            d = pi_2
        else:
            d = math.atan(pixle[1] / float(pixle[0]))
        for index in range(density_d):
            hough[index][int((r * math.cos(index * d_step - d)) // r_step)].append(pixle)

    angle = []
    for d in range(density_d):
        lines = []
        parallel = []
        for r in range(density_r * 2):
            if len(hough[d][r]) > line_inf:
                for pixle in hough[d][r]:
                    index = pixle[0] + pixle[1] * width
                    if in_array[index] != 2:
                        in_array[index] = 3
                for line in lines:
                    parallel.append([r - line[0], line[0], r, line[1], hough[d][r]])
                lines.append([r, hough[d][r]])
        angle.append(parallel)

    sides = None
    max = 0
    for ind1, ind2 in enumerate(range(density_d / 2, density_d)):
        for par1 in angle[ind1]:
            for par2 in angle[ind2]:
                if abs(par1[0] - par2[0]) < a_threas and 300 > par1[0] + par2[0] > 40:
                    value = len(par1[3]) + len(par1[4]) + len(par2[3]) + len(par2[4])
                    value /= float(par1[0] + par2[0])
                    if value > max:
                        value = max
                        sides = [ind1, par1, par2]

    if sides != None:
        for pixle in sides[1][3]: 
            index = pixle[0] + pixle[1] * width
            if in_array[index] != 2:
                in_array[index] = 4
        for pixle in sides[1][4]:  
            index = pixle[0] + pixle[1] * width
            if in_array[index] != 2:
                in_array[index] = 4
        for pixle in sides[2][3]:  
            index = pixle[0] + pixle[1] * width
            if in_array[index] != 2:
                in_array[index] = 4
        for pixle in sides[2][4]:  
            index = pixle[0] + pixle[1] * width
            if in_array[index] != 2:
                in_array[index] = 4
        square_radius = int(sides[1][0] + sides[2][0] / 2)
        square_angle = int(sides[0] * d_step)
        dist1 = int((sides[1][1] + sides[1][2]) / 2)
        dist2 = int((sides[2][1] + sides[2][2]) / 2)
        square_x = dist1 * math.cos(square_angle) + dist2 * math.sin(square_angle)
        square_y = dist1 * math.sin(square_angle) + dist2 * math.cos(square_angle)

    print square_x

    print 'hough square detector spend', time.time() - start, 'seconds'

    return square_x, square_y, square_radius, square_angle



