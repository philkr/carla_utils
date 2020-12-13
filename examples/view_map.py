import pathlib
import carla_utils.map as carla_map

import matplotlib.pyplot as plt
import numpy as np


def check_import(xodr_path):
    cmap = carla_map.Map('unused', pathlib.Path(xodr_path).read_text())

    points = [x.transform.location for x in cmap.generate_waypoints(10)]
    points = np.array([(point.x, point.y) for point in points])

    plt.plot(points[:, 0], points[:, 1], 'r.')
    plt.show()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('xodr_path', type=pathlib.Path)

    check_import(**vars(parser.parse_args()))
