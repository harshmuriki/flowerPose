import numpy as np

def kinematics2D(r, plant_x, plant_y, angle):
    # r  #mm
    # all the angles for only angle
    # for temp_angle in range(min_angle, max_angle, step_degree):
        # x axis movement

    rad = (angle * (2*np.pi)) / 360
    x = plant_x + r*np.cos(rad)

    # y axis movement
    y = plant_y + r*np.sin(rad)

    # arm angle change
    # in code

    return x, y

def kinematics3D(r, plant_x, plant_y, plant_z, theta, phi):
    theta = (theta * (2*np.pi)) / 360
    phi = (phi * (2*np.pi)) / 360
    x = plant_x + r*np.sin(phi)*np.cos(theta)
    y = plant_y + r*np.sin(phi)*np.sin(theta)
    z = plant_z + r*np.cos(phi)

    return x, y, z
