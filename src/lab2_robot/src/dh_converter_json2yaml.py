import json
from tf.transformations import *

x_axis, y_axis, z_axis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


params = {}

with open('dh_data.json', 'r') as file:
    params = json.loads(file.read())

with open('yaml_data_dh.yaml', 'w') as file:
    for key in params.keys():
        a, d, alpha, theta = params[key]
        alpha, a, d, theta = float(alpha), float(a), float(d), float(theta)

        trans_x = translation_matrix((a, 0, 0))
        rot_x = rotation_matrix(alpha, x_axis)
        rot_z = rotation_matrix(theta, z_axis)
        trans_z = translation_matrix((0, 0, d))

        matrix = concatenate_matrices(trans_x, rot_x, rot_z, trans_z)

        rpy = euler_from_matrix(matrix)
        xyz = translation_from_matrix(matrix)

        file.write(key + ":\n")
        file.write("  j_xyz: {} {} {}\n".format(*xyz))
        file.write("  j_rpy: {} {} {}\n".format(*rpy))
        file.write("  l_xyz: 0 0 {}\n".format(-d / 2))
        file.write("  l_rpy: 0 0 0\n")
        file.write("  l_len: {}\n".format(d))
