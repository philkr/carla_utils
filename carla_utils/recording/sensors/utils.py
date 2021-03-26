import numpy as np

from scipy.spatial.transform import Rotation


def transform_to_matrix(transform):
    x = transform.location.x
    y = transform.location.y
    z = transform.location.z

    a = np.radians(transform.rotation.yaw)
    b = np.radians(transform.rotation.pitch)
    g = np.radians(transform.rotation.roll)

    c_a, c_b, c_g = np.cos(a), np.cos(b), np.cos(g)
    s_a, s_b, s_g = np.sin(a), np.sin(b), np.sin(g)

    mat = np.float32([
        [c_a * c_b,  c_a * s_b * s_g - s_a * c_g,  c_a * s_b * c_g + s_a * s_g, x],
        [s_a * c_b,  s_a * s_b * s_g + c_a * c_g,  s_a * s_b * c_g - c_a * s_g, y],
        [     -s_b,                    c_b * s_g,                    c_b * c_g, z],
        [        0,                            0,                            0, 1],
    ])

    mat[1, :] *= -1.0
    mat[:, 1] *= -1.0

    return mat
