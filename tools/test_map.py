# Compares carla and carla_utils map
from contextlib import contextmanager
import numpy as np
import random

from carla_utils import map as utils
import carla

@contextmanager
def timeit(name=None):
    from time import time
    t0 = time()
    yield
    t1 = time()
    print('{:30} {:0.5f}'.format(name, t1-t0))


def cast(value, type_):
    if isinstance(value, (carla.Location, utils.Location)):
        return __import__(type_.__module__).Location(value.x, value.y, value.z)
    return value


class Test:
    def __init__(self, methods, *args, **kwargs):
        self.objects = []
        for k, v in methods.items():
            with timeit('{}.{}'.format(k, v.__name__)):
                self.objects.append((k, v(*args, **kwargs)))

    def __getattr__(self, fn):
        def bench(*args, verbose=True, **kwargs):
            r = {}
            for k, v in self.objects:
                f = getattr(v, fn)
                cast_args = [cast(a, v) for a in args]
                if verbose:
                    with timeit('{}.{}'.format(k, fn)):
                        r[k] = f(*cast_args, **kwargs)
                else:
                    r[k] = f(*cast_args, **kwargs)
            return r
        return bench


def compare_list(a, b, cmp):
    if len(a) != len(b):
        print('length mismatch {} vs {}'.format(len(a), len(b)))
    else:
        for i, j in zip(a, b):
            if not cmp(i, j):
                break


def compare_transform(a, b, eps=1e-4):
    la, lb = a.transform.location, b.transform.location
    if any([abs(getattr(la, n) - getattr(lb, n)) > eps for n in ['x', 'y', 'z']]):
        print('Location mismatch {:0.2f} {:0.2f} {:0.2f}    {:0.2f} {:0.2f} {:0.2f}'.format(la.x, la.y, la.z, lb.x, lb.y, lb.z))
        return False

    ra, rb = a.transform.rotation, b.transform.rotation
    if any([abs(getattr(ra, n) - getattr(rb, n)) > eps for n in ['pitch', 'yaw', 'roll']]):
        print('Rotation mismatch {:0.2f} {:0.2f} {:0.2f}    {:0.2f} {:0.2f} {:0.2f}'.format(ra.pitch, ra.yaw, ra.roll, rb.pitch, rb.yaw, rb.roll))
        return False


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('map')
    args = parser.parse_args()

    with open(args.map) as f:
        xodr = f.read()

    t = Test(dict(carla=carla.Map, utils=utils.Map), 'test', xodr)

    # Test waypoint generation
    wps = t.generate_waypoints(1)
    compare_list(*wps.values(), compare_transform)

    # Test topology generation
    topo = t.get_topology()
    compare_list(*topo.values(), lambda a, b: compare_list(a, b, compare_transform))

    # Generate some random test points for the get_waypoint query
    random_points = []
    for noise_level in np.linspace(0, 10, 1000000):
        l = random.choice(list(wps.values())[0]).transform.location
        l.x += noise_level * (random.random()-0.5)
        l.y += noise_level * (random.random()-0.5)
        l.z += noise_level * (random.random()-0.5)
        random_points.append(l)

    print('# points', len(random_points))
    for project_to_road in [True, False]:
        print('project_to_road =', project_to_road)
        t.get_waypoint(random_points[0], project_to_road=project_to_road)

        n_fail, n_win = 0, 0
        for p in random_points:
            r = t.get_waypoint(p, project_to_road=project_to_road, verbose=False)
            if r['carla'] is None or r['utils'] is None:
                continue
            dst = {k: np.sqrt((p.x-l.x)**2+(p.y-l.y)**2+(p.z-l.z)**2) for k, w in r.items() for l in [w.transform.location]}
            if dst['carla'] + 0.1 < dst['utils']:
                if n_fail == 0:
                    print(dst)
                    compare_transform(*r.values())
                    print([(v.road_id, v.lane_id, v.s) for v in r.values()])
                n_fail += 1
            if dst['utils'] + 0.1 < dst['carla']:
                n_win += 1

        print("%fail = {}   {} / {}".format(100*n_fail / len(random_points), n_fail, len(random_points)))
        print("%win  = {}   {} / {}".format(100*n_win / len(random_points), n_win, len(random_points)))


if __name__ == "__main__":
    main()
