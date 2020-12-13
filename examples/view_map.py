import pathlib
import carla_utils.map as carla_map

import matplotlib.pyplot as plt
import numpy as np


def check_import(xodr_path):
    cmap = carla_map.Map('unused', pathlib.Path(xodr_path).read_text())

    road = [x.transform.location for x in cmap.generate_waypoints(10)]
    road = np.array([(r.x, r.y) for r in road])

    plt.plot(road[:, 0], road[:, 1], 'r.')
    plt.show()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('xodr_path', type=pathlib.Path)

    check_import(**vars(parser.parse_args()))
