#!/usr/bin/env python



import numpy as np
import math
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from skimage.measure import LineModelND, ransac


xyz = np.array([[-0.0233141, -0.0178063, 1.54962],
              [-0.0622198, -0.100469, 1.49478],
              [0.281667, -0.0627161, 1.46354],
              [0.131184, -0.0348248, 1.21341],
              [-0.290787, -0.0973173, 1.08952],
              [0.245746, -0.0662267, 1.11599],
              [-0.0594294, -0.288045, 1.17382],
              [-0.268843, -0.170753, 1.10177],
              [-0.217578, -0.209957, 1.12108],
              [0.164148, -0.392932, 1.10008],
              [0.298818, -0.310998, 1.13982],
              [0.313253, -0.351796, 1.21595],
              [0.333587, -0.354537, 1.24463],
              [0.461787, -0.283944, 1.23024],
              [0.22751, -0.232413, 1.27101],
              [0.48105, -0.16478, 1.26465],
              [0.411901, -0.184881, 1.27819],
              [0.504287, -0.164516, 1.17278],
              [0.445916, -0.0653491, 1.22861],
              [0.424359, -0.083734, 1.26391],
              [0.242117, -0.104357, 1.29507],
              [0.295311, -0.0315815, 1.25837],
              [0.289163, -0.0348039, 1.27414],
              [0.292514, 0.0916709, 1.31655],
              [0.183616, 0.135501, 1.30206],
              [0.167484, 0.177048, 1.31427],
              [0.183079, 0.265563,  1.18023],
              [-0.290176, 0.106768, 1.23873],
              [-0.133456, 0.0866687, 1.21802],
              [-0.260372, 0.0319368, 1.1148],
              [-0.123193, 0.162377, 1.21021],
              [-0.169999, 0.198317, 1.12971],
              [-0.0225, 0.196541, 1.21751],
              [-0.375585, -0.0446989, 1.0402],
              [-0.193917, -0.0699643, 1.13882],
              [-0.39402, -0.133385, 1.11074],
              [0.0852066, -0.239025, 1.55542],
              [0.479761, -0.387189, 1.55347],
              [-0.458955, -0.228883, 1.4289],
              [-0.200584, -0.280572, 1.5139],
              [0.572202, -0.264296, 1.4837],
              [0.588874, -0.225122, 1.44479],
              [0.461536, -0.182095, 1.55737],
              [0.639481, -0.217868, 1.42102],
              [0.534107, -0.23362, 1.48005],
              [0.462094, -0.259057, 1.50593],
              [0.353456, -0.139633, 1.47502],
              [0.243982, -0.237548, 1.4466],
              [0.458517, 0.230657, 1.26043],
              [0.629461, 0.0997608, 1.25244],
              [0.100971, -0.0169415, 1.55952],
              [0.579051, 0.11906, 1.38485],
              [-0.0363949, -0.306579, 2.69037],
              [-0.0195847, -0.349931, 2.67688],
              [0.763438, -0.319243, 2.63864],
              [0.38468, -0.474052, 2.77374],
              [0.183752, -0.383089, 2.80229],
              [0.339071, -0.264018, 2.64799],
              [0.71315, -0.332008, 2.77588],
              [0.0829299, -0.476977,  2.85322],
              [-0.596457, -0.452046, 2.14856],
              [0.477649, -0.332391, 2.19899],
              [0.628936, -0.175275, 2.06614],
              [0.534427, -0.621499, 2.04679],
              [0.272012, -0.273767, 2.2439],
              [-0.554142, -0.12614, 2.05613],
              [0.812122, -0.135928, 2.07138],
              [0.617972, -0.0785436, 2.18298],
              [0.958026, -0.25164, 1.96235],
              [0.883592, 0.0326147, 2.10328],
              [-0.139921, -0.642485, 2.82907],
              [0.721158, -0.471845, 2.52089],
              [-0.594951, -0.700265, 2.45982],
              [-0.630618, -0.522617, 2.52868],
              [0.26055, -0.566313, 2.6209],
              [0.730358, -0.489606, 2.44699],
              [0.394614, -0.851993, 3.43425],
              [-0.373792, -0.340206, 1.59005],
              [0.472688, -0.22003, 1.52748],
              [0.408056, -0.173156, 1.38107],
              [-0.278764, -0.217256, 1.28577],
              [0.152461, -0.168146, 1.47508],
              [-0.185008, -0.197835, 1.4923],
              [-0.379361, -0.214942, 1.57446],
              [0.452466, -0.129796, 1.59966],
              [0.509455, -0.128324, 1.82801],
              [0.520782, -0.111744, 1.85086],
              [-0.484451, -0.261893, 1.82778],
              [0.683946, -0.371487, 2.01492],
              [-0.356848, -0.44292, 2.1378],
              [-0.466888, -0.415688, 2.08639],
              [-0.429429, -0.440058, 2.07016],
              [0.172515, -0.495504, 2.12016],
              [0.775713, -0.411456, 2.01163],
              [-0.435831, -0.576068, 2.13112],
              [0.0740378, -0.630959, 2.61727],
              [0.67502, -0.556685, 2.37712],
              [-0.173521, -1.23108, 3.33828]])



# np.random.seed(seed=1)
#
# # generate coordinates of line
# point = np.array([0, 0, 0], dtype='float')
# direction = np.array([1, 1, 1], dtype='float') / np.sqrt(3)
# xyz = point + 10 * np.arange(-100, 100)[..., np.newaxis] * direction
#
# # add gaussian noise to coordinates
# noise = np.random.normal(size=xyz.shape)
# xyz += 0.5 * noise
# xyz[::2] += 20 * noise[::2]
# xyz[::4] += 100 * noise[::4]

# robustly fit line only using inlier data with RANSAC algorithm
model_robust, inliers = ransac(xyz, LineModelND, min_samples=2,
                               residual_threshold=1, max_trials=1000)
outliers = inliers == False

print(model_robust.params)
xs = np.array([model_robust.params[0][0], model_robust.params[1][0]])
ys = np.array([model_robust.params[0][1], model_robust.params[1][1]])
zs = np.array([model_robust.params[0][2], model_robust.params[1][2]])

# normalize axis of cone
axis = model_robust.params[0] - model_robust.params[1]
axis = axis / np.linalg.norm(axis)

mean_xyz = np.mean(xyz, 0)
print(mean_xyz)

v_guess = np.array([-0.14, 0.06, 0.05])
axis = mean_xyz - v_guess;
axis = axis /np.linalg.norm(axis)
print(axis)

v_init = v_guess

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(v_init[0], v_init[1], v_init[2], c='g', marker='x')


vertex_update = 0
points_outside = False
moving_forward = True

move_vertex = True

# move vertex up or down the axis for best current fit
while move_vertex:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - v_init, axis)
        cone_radius = cone_dist * math.tan(0.261799)
        orth_vect = np.dot(i - v_init, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + v_init - i
        orth_dist = np.linalg.norm(orth_vect)
        if orth_dist > cone_radius:
            points_outside = True

    if points_outside == True:
        v_init = v_init - 0.01 * axis
        if vertex_update == 0:
            moving_forward = False
        if vertex_update > 0 and moving_forward:
            move_vertex = False
            break
        vertex_update = vertex_update + 1
    else:
        if vertex_update == 0:
            moving_forward = True
        if vertex_update > 0 and not moving_forward:
            move_vertex = False
            break
        v_init = v_init + 0.01 * axis
        vertex_update = vertex_update + 1

ax.scatter(v_init[0], v_init[1], v_init[2], c='g', marker='x')

cone_error = 0

# find data point closest to edge of cone
closest_edge_point = 0
closest_edge_dist = 100
closest_edge_height = 0
for i in xyz:
    cone_dist = np.dot(i - v_init, axis)
    cone_radius = cone_dist * math.tan(0.261799)
    orth_vect = np.dot(i - v_init, axis) / np.dot(axis, axis)
    orth_vect = orth_vect * axis + v_init - i
    orth_dist = np.linalg.norm(orth_vect)

    cone_error = cone_error + cone_radius - orth_dist

    if cone_radius - orth_dist < closest_edge_dist:
        closest_edge_point = i
        closest_edge_dist = cone_radius - orth_dist
        closest_edge_height = cone_dist

print("closest edge point")
print(closest_edge_point)

print("cone error: ")
print(cone_error)


# calculate new v_init
new_axis_point = v_init + closest_edge_height * axis
change_vector = new_axis_point - closest_edge_point
print("new direction of change")
print(change_vector)
v_init = v_init + 0.01 * change_vector



vertex_update = 0
points_outside = False
moving_forward = True

move_vertex = True

# move vertex up or down the axis for best current fit
while move_vertex:
    points_outside = False
    for i in xyz:
        cone_dist = np.dot(i - v_init, axis)
        cone_radius = cone_dist * math.tan(0.261799)
        orth_vect = np.dot(i - v_init, axis) / np.dot(axis, axis)
        orth_vect = orth_vect * axis + v_init - i
        orth_dist = np.linalg.norm(orth_vect)
        if orth_dist > cone_radius:
            points_outside = True

    if points_outside == True:
        v_init = v_init - 0.01 * axis
        if vertex_update == 0:
            moving_forward = False
        if vertex_update > 0 and moving_forward:
            move_vertex = False
            break
        vertex_update = vertex_update + 1
    else:
        if vertex_update == 0:
            moving_forward = True
        if vertex_update > 0 and not moving_forward:
            move_vertex = False
            break
        v_init = v_init + 0.01 * axis
        vertex_update = vertex_update + 1

ax.scatter(v_init[0], v_init[1], v_init[2], c='g', marker='x')


cone_error = 0

# find data point closest to edge of cone
closest_edge_point = 0
closest_edge_dist = 100
closest_edge_height = 0
for i in xyz:
    cone_dist = np.dot(i - v_init, axis)
    cone_radius = cone_dist * math.tan(0.261799)
    orth_vect = np.dot(i - v_init, axis) / np.dot(axis, axis)
    orth_vect = orth_vect * axis + v_init - i
    orth_dist = np.linalg.norm(orth_vect)

    cone_error = cone_error + cone_radius - orth_dist

    if cone_radius - orth_dist < closest_edge_dist:
        closest_edge_point = i
        closest_edge_dist = cone_radius - orth_dist
        closest_edge_height = cone_dist

print("closest edge point")
print(closest_edge_point)


print("cone error: ")
print(cone_error)


# calculate new v_init
new_axis_point = v_init + closest_edge_height * axis
change_vector = new_axis_point - closest_edge_point
print(new_axis_point)
print(change_vector)
v_init = v_init - 0.01 * change_vector



ax.scatter(xyz[inliers][:, 0], xyz[inliers][:, 1], xyz[inliers][:, 2], c='b',
           marker='o', label='Inlier data')
ax.scatter(xyz[outliers][:, 0], xyz[outliers][:, 1], xyz[outliers][:, 2], c='r',
           marker='o', label='Outlier data')
ax.scatter(v_init[0], v_init[1], v_init[2], c='g', marker='x')
ax.plot(np.array([v_init[0], mean_xyz[0]]), np.array([v_init[1], mean_xyz[1]]), np.array([v_init[2], mean_xyz[2]]))
ax.plot(xs, ys, zs)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend(loc='lower left')
plt.show()
