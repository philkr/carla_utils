# Compares carla and carla_utils map
from contextlib import contextmanager


@contextmanager
def timeit(name=None):
    from time import time
    t0 = time()
    yield
    t1 = time()
    print('{:30} {:0.5f}'.format(name, t1-t0))


class Test:
    def __init__(self, methods, *args, **kwargs):
        self.objects = []
        for k, v in methods.items():
            with timeit('{}.{}'.format(k, v.__name__)):
                self.objects.append((k, v(*args, **kwargs)))

    def __getattr__(self, fn):
        def bench(*args, **kwargs):
            r = {}
            for k, v in self.objects:
                f = getattr(v, fn)
                with timeit('{}.{}'.format(k, fn)):
                    r[k] = f(*args, **kwargs)
            return r
        return bench


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('map')
    args = parser.parse_args()

    with open(args.map) as f:
        xodr = f.read()

    from carla_utils import map as utils
    import carla

    t = Test(dict(carla=carla.Map, utils=utils.Map), 'test', xodr)
    t.generate_waypoints(0.1)
    t.get_topology()
    # with timeit('carla.Map'):
    #     carla_map = carla.Map('test', xodr)
    #
    # with timeit('util.Map'):
    #     utils_map = map.Map('test', xodr)
    #
    # with timeit('carla_map.generate_waypoints'):
    #     carla_waypoints = carla_map.generate_waypoints(0.5)
    #
    # with timeit('utils_map.generate_waypoints'):
    #     utils_waypoints = utils_map.generate_waypoints(0.5)



if __name__ == "__main__":
    main()
