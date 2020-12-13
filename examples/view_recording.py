"""
Testing the parsing function

Press 'q' to quit.
"""
import numpy as np
import matplotlib.pyplot as plt

from carla_utils.recording import parse


try:
    from tqdm import tqdm
except ImportError:
    def tqdm(x, *args, **kwargs):
        return x


def main(recording):
    cmap, frames = parse(recording)

    plt.ion()

    road = [x.transform.location for x in cmap.generate_waypoints(2.5)]
    road = np.array([(r.x, r.y) for r in road])

    plt.plot(road[:, 0], road[:, 1], 'r.')
    plt.pause(1e-5)

    vehicle_points = plt.plot([], [], 'b+')[0]

    for f in tqdm(frames):
        vehicles = np.array([v.location[:2] for v in f.vehicles])
        vehicle_points.set_data(vehicles[:, 0], vehicles[:, 1])

        plt.pause(1e-5)


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('recording')

    main(**vars(parser.parse_args()))
