#!/usr/bin/env python

import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

data_files = ['output_6_all_points.xml', 'output_7_all_points.xml', 'output_8_all_points.xml',
                'output_9_all_points.xml', 'output_10_all_points.xml']

confidence_level = 100

xyz = []
dist = []

echosounder_position = []


def optimize_position(P_0):
    global xyz
    error_vector = np.array([0.0, 0.0, 0.0])
    for i in range(np.size(xyz,0)):
        X_i = np.array(xyz[i])
        point_vector = X_i - P_0
        magnitude_vector = (np.linalg.norm(point_vector) - dist[i]) * point_vector / np.linalg.norm(point_vector)
        error_vector = np.add(error_vector, magnitude_vector)
    error_magnitude = np.linalg.norm(error_vector)
    return error_magnitude / np.size(xyz,0)


def get_echosounder_position():
    # Optimize echosounder position
    echosounder_initial_position = np.array([0.0, 0.0, 0.0])
    res = minimize(optimize_position, echosounder_initial_position, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})
    echosounder_position = res.x

    print("\nOptimize position:\n" + str(res))
    print("\nEchosounder position: " + str(echosounder_position) + "\n")

    return echosounder_position


def optimize_direction(axis):
    axis = axis / np.linalg.norm(axis)
    points_outside = True
    angle = 0.0
    while points_outside:
        points_outside = False
        for i in xyz:
            cone_dist = np.dot(i - echosounder_position, axis)
            cone_radius = cone_dist * math.tan(angle)
            orth_vect = np.dot(i - echosounder_position, axis) / np.dot(axis, axis)
            orth_vect = orth_vect * axis + echosounder_position - i
            orth_dist = np.linalg.norm(orth_vect)
            if orth_dist > cone_radius:
                angle = angle + 0.01
                points_outside = True
    return angle


def get_echosounder_direction():
    # Initial echosounder direction axis
    init_direction_vector = np.array([0.0, 0.0, 0.0])
    for i in range(np.size(xyz,0)):
        unit_data_vect = (xyz[i] - echosounder_position) / np.linalg.norm(xyz[i] - echosounder_position)
        init_direction_vector = init_direction_vector + unit_data_vect
    init_direction_vector = init_direction_vector / np.size(xyz,0)

    # Optimize echosounder directional vector
    res = minimize(optimize_direction, init_direction_vector, method='nelder-mead', options={'xatol': 1e-8, 'disp': True})
    echosounder_direction = res.x

    print("\nOptimize direction:\n" + str(res))
    print("\nEchosounder direction: " + str(echosounder_direction))
    print("\nFinal echosounder angle: " + str(res.fun))

    return echosounder_direction


def get_data_points():
    global xyz, dist
    # Read in 3-D point data
    for file in data_files:
        file = "../" + file
        tree = ET.parse(file)
        root = tree.getroot()
        for child in root:
            if float(child.get('confidence')) >= confidence_level:
                xyz.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])
                dist.append(child.get('depth'))
    xyz = np.array(xyz)
    dist = np.array(dist).astype(float)


def plot_data(echosounder_position, echosounder_direction):
    # Illustrate echosounder's directional vector
    axis_line_x = [echosounder_position[0], echosounder_position[0] + 6 * echosounder_direction[0]]
    axis_line_y = [echosounder_position[1], echosounder_position[1] + 6 * echosounder_direction[1]]
    axis_line_z = [echosounder_position[2], echosounder_position[2] + 6 * echosounder_direction[2]]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], c='b', marker='o')
    ax.scatter(echosounder_position[0], echosounder_position[1], echosounder_position[2], c='r', marker='x')
    ax.plot(axis_line_x, axis_line_y, axis_line_z, c="g")

    plt.title('Echo Sounder Data')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.legend(loc='lower left')

    plt.show()


if __name__ == "__main__":
    get_data_points()

    echosounder_position = get_echosounder_position()

    echosounder_direction = get_echosounder_direction()

    plot_data(echosounder_position, echosounder_direction)
