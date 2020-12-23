from carla_utils import map as utils
from tools.test_map import Test

import carla
import matplotlib.pyplot as plt
import numpy as np


def generate_waypoints(t):
    test = t.generate_waypoints(1)

    # ignore the z.
    gt = np.float32([(l.x, l.y) for l in map(lambda x: x.transform.location, test['carla'])])
    pred = np.float32([(l.x, l.y) for l in map(lambda x: x.transform.location, test['utils'])])

    origin = gt[np.random.randint(0, len(gt))]

    plt.xlim(origin[0] - 25, origin[0] + 25)
    plt.ylim(origin[1] - 25, origin[1] + 25)

    plt.plot(gt[:, 0], gt[:, 1], 'ro')
    plt.plot(pred[:, 0], pred[:, 1], 'b.')
    plt.show()



def get_waypoint(t, project_to_road):
    initial = t.generate_waypoints(1)['carla']
    origin = np.random.choice(initial).transform.location
    gt = np.float32([(l.x, l.y) for l in map(lambda x: x.transform.location, initial)])

    plt.xlim(origin.x - 25, origin.x + 25)
    plt.ylim(origin.y - 25, origin.y + 25)
    plt.plot(gt[:, 0], gt[:, 1], 'r.')

    for noise_level in np.linspace(0, 25, 10):
        x = origin.x + noise_level * (2 * np.random.rand() - 1)
        y = origin.y + noise_level * (2 * np.random.rand() - 1)
        z = origin.z + noise_level * (2 * np.random.rand() - 1)
        random = carla.Location(x, y, z)

        test = t.get_waypoint(random, project_to_road=project_to_road, verbose=False)

        plt.plot([random.x, test['carla'].transform.location.x], [random.y, test['carla'].transform.location.y], 'r-')
        plt.plot([random.x, test['utils'].transform.location.x], [random.y, test['utils'].transform.location.y], 'b--')

    plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('map')
    args = parser.parse_args()

    with open(args.map) as f:
        xodr = f.read()

    t = Test(dict(carla=carla.Map, utils=utils.Map), 'test', xodr)

    generate_waypoints(t)
    get_waypoint(t, project_to_road=True)


if __name__ == "__main__":
    main()
