#! /usr/bin/python

import json


if __name__ == '__main__':
    params = {}
    with open('../config/dh_table.json', 'r') as file:
        params = json.loads(file.read())

    with open('../config/urdf_params.yaml', 'w') as file:
        for key in params.keys():
            a, d, alfa, theta = params[key]
            a, d, alfa, theta =  float(a), float(d), float(alfa), float(theta)

            file.write(key + ":\n")
            file.write("  joint_xyz: %s 0 %s\n" % (a, d))
            file.write("  joint_rpy: %s 0 0\n" % (alfa))
            file.write("  link_x_size: %s\n" % (a))
            file.write("  link_xyz: %s 0 0\n" % (a/2))
            file.write("  link_rpy: 0 0 0\n")

