#!/usr/bin/env python

import xml.etree.ElementTree as ET
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skimage.measure import LineModelND, ransac
import math

tree = ET.parse('output_6_all_points.xml')
root = tree.getroot()

confidence_level = 100
low_confidence_level = 30

xs = []
ys = []
zs = []
xyz = []
low_perc = []
dist = []

for child in root:
    if float(child.get('confidence')) >= confidence_level:
        xs.append(child.get('x'))
        ys.append(child.get('y'))
        zs.append(child.get('z'))
        xyz.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])
        dist.append(child.get('depth'))
    elif float(child.get('confidence')) < low_confidence_level:
        low_perc.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])

tree = ET.parse('output_7_all_points.xml')
root = tree.getroot()

for child in root:
    if float(child.get('confidence')) >= confidence_level:
        xs.append(child.get('x'))
        ys.append(child.get('y'))
        zs.append(child.get('z'))
        xyz.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])
        dist.append(child.get('depth'))
    elif float(child.get('confidence')) < low_confidence_level:
        low_perc.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])

tree = ET.parse('output_8_all_points.xml')
root = tree.getroot()

for child in root:
    if float(child.get('confidence')) >= confidence_level:
        xs.append(child.get('x'))
        ys.append(child.get('y'))
        zs.append(child.get('z'))
        xyz.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])
        dist.append(child.get('depth'))
    elif float(child.get('confidence')) < low_confidence_level:
        low_perc.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])

tree = ET.parse('output_9_all_points.xml')
root = tree.getroot()

for child in root:
    if float(child.get('confidence')) >= confidence_level:
        xs.append(child.get('x'))
        ys.append(child.get('y'))
        zs.append(child.get('z'))
        xyz.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])
        dist.append(child.get('depth'))
    elif float(child.get('confidence')) < low_confidence_level:
        low_perc.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])

tree = ET.parse('output_10_all_points.xml')
root = tree.getroot()

for child in root:
    if float(child.get('confidence')) >= confidence_level:
        xs.append(child.get('x'))
        ys.append(child.get('y'))
        zs.append(child.get('z'))
        xyz.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])
        dist.append(child.get('depth'))
    elif float(child.get('confidence')) < low_confidence_level:
        low_perc.append([float(child.get('x')), float(child.get('y')), float(child.get('z'))])


xs = np.array(xs).astype(float)
ys = np.array(ys).astype(float)
zs = np.array(zs).astype(float)
xyz = np.array(xyz)
low_perc = np.array(low_perc)
dist = np.array(dist).astype(float)

print(xyz)

# model_robust, inliers = ransac(xyz, LineModelND, min_samples=1000,
#                                residual_threshold=0.01, max_trials=5000)
# outliers = inliers == False
#
# print(model_robust.params)
#
# x_ransac = np.array([model_robust.params[0][0], model_robust.params[1][0]])
# y_ransac = np.array([model_robust.params[0][1], model_robust.params[1][1]])
# z_ransac = np.array([model_robust.params[0][2], model_robust.params[1][2]])

vertex = np.array([-0.14, 0.06, 0.05])

for x in range(2):
    move_to = np.array([0.0, 0.0, 0.0])

    for i in range(xs.size):
        data_vect = xyz[i] - vertex
        cur_dist = np.linalg.norm(data_vect)
        unit_data_vect = data_vect / cur_dist

        if (cur_dist > dist[i]):
            move_to = move_to + (cur_dist - dist[i]) * unit_data_vect
        elif (cur_dist < dist[i]):
            move_to = move_to - (dist[i] - cur_dist) * unit_data_vect

    print("move to")
    print(move_to)

    move_to = move_to / xs.size
    print("move to")
    print(move_to)

    dist_to_new = np.linalg.norm(move_to)
    print(dist_to_new)

    vertex  = vertex + move_to


print("ECHO SOUNDER POSITION:")
print(vertex)

axis = np.array([0.0, 0.0, 0.0])
for i in range(xs.size):
    unit_data_vect = (xyz[i] - vertex) / np.linalg.norm(xyz[i] - vertex)
    axis = axis + unit_data_vect
axis = axis / xs.size
print("current axis:")
print(axis)


points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        # print("point distance")
        # print(np.linalg.norm(i-vertex))
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle")
print(angle)

points_inside = True
while points_inside:
    points_inside = False
    for i in low_perc:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist < cone_radius:
            angle = angle - 0.01
            points_inside = True

print(low_perc)
print("angle")
print(angle)

old_axis = axis
axis = old_axis + np.array([old_axis[0] + 0.1, old_axis[1], old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 1")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 1")
print(angle)


axis = old_axis + np.array([old_axis[0] - 0.1, old_axis[1], old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 2")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 2")
print(angle)



#
# axis = old_axis + np.array([old_axis[0], old_axis[1] + 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 3")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 3")
# print(angle)
#
#
#
#
# axis = old_axis + np.array([old_axis[0], old_axis[1] - 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 4")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 4")
# print(angle)




# KEEPING ANGLE 2
print("KEEPING ANGLE 2")



old_axis = axis
axis = old_axis + np.array([old_axis[0] + 0.1, old_axis[1], old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 1")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 1")
print(angle)



axis = old_axis + np.array([old_axis[0] - 0.1, old_axis[1], old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 2")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 2")
print(angle)


# print(axis)
axis = old_axis + np.array([old_axis[0], old_axis[1] + 0.1, old_axis[2]])
# print(axis)
axis = axis / np.linalg.norm(axis)

print("new axis 3")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 3")
print(angle)



# axis = old_axis + np.array([old_axis[0], old_axis[1] - 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 4")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 4")
# print(angle)
#
#
#
#
# axis = old_axis + np.array([old_axis[0] + 0.1, old_axis[1] + 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 5")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 5")
# print(angle)
#
#
#
#
# axis = old_axis + np.array([old_axis[0] + 0.1, old_axis[1] - 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 6")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 6")
# print(angle)
#
#
#
# axis = old_axis + np.array([old_axis[0] - 0.1, old_axis[1] + 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 7")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 7")
# print(angle)
#
#
#
# axis = old_axis + np.array([old_axis[0] - 0.1, old_axis[1] - 0.1, old_axis[2]])
# axis = axis / np.linalg.norm(axis)
#
# print("new axis 8")
# print(axis)
#
# points_outside = True
# angle = 0.261799
# while points_outside:
#     points_outside = False
#     for i in xyz:
#         cone_dist = np.dot(i - vertex, axis)
#         # print("cone distance")
#         # print(cone_dist)
#         cone_radius = cone_dist * math.tan(angle)
#         # print("cone radius")
#         # print(cone_radius)
#         orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
#         orth_vect = orth_vect * axis + vertex - i
#         orth_dist = np.linalg.norm(orth_vect)
#         # print("orth distance")
#         # print(orth_dist)
#         if orth_dist > cone_radius:
#             angle = angle + 0.01
#             points_outside = True
#
# print("angle 8")
# print(angle)





# KEEPING ANGLE 3
print("KEEPING ANGLE 3")



old_axis = axis
axis = old_axis + np.array([old_axis[0] + 0.1, old_axis[1], old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 1")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 1")
print(angle)


axis = old_axis + np.array([old_axis[0] - 0.1, old_axis[1], old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 2")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 2")
print(angle)



axis = old_axis + np.array([old_axis[0], old_axis[1] + 0.1, old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 3")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 3")
print(angle)


axis = old_axis + np.array([old_axis[0], old_axis[1] - 0.1, old_axis[2]])
axis = axis / np.linalg.norm(axis)

print("new axis 4")
print(axis)

points_outside = True
angle = 0.261799
while points_outside:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist > cone_radius:
            angle = angle + 0.01
            points_outside = True

print("angle 4")
print(angle)


print("final axis")
print(axis)


points_inside = True
while points_inside:
    points_inside = False
    for i in low_perc:
        cone_dist = np.dot(i - vertex, axis)
        # print("cone distance")
        # print(cone_dist)
        cone_radius = cone_dist * math.tan(angle)
        # print("cone radius")
        # print(cone_radius)
        orth_vect = np.dot(i - vertex, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + vertex - i
        orth_dist = np.linalg.norm(orth_vect)
        # print("orth distance")
        # print(orth_dist)
        if orth_dist < cone_radius:
            angle = angle - 0.01
            points_inside = True


axis_line_x = [vertex[0], vertex[0] + 6 * axis[0]]
axis_line_y = [vertex[1], vertex[1] + 6 * axis[1]]
axis_line_z = [vertex[2], vertex[2] + 6 * axis[2]]

print("low conf angle")
print(angle)
print(xs.size)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(xs, ys, zs, c='b', marker='o')
ax.scatter(vertex[0], vertex[1], vertex[2], c='r', marker='x')
ax.plot(axis_line_x, axis_line_y, axis_line_z, c="g")

plt.title('Echo Sounder Data')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.legend(loc='lower left')

plt.show()
