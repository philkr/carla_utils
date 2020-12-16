import time

from contextlib import contextmanager
from .camera import Camera


MAPPING = {
        'sensor.camera.rgb': Camera
        }


@contextmanager
def sensor_pool(world, target_id, sensor_dict):
    """
    Spawns and cleans up after sensors.
    """
    target_actor = world.get_actor(target_id)
    blueprints = world.get_blueprint_library()

    pool = list()

    try:
        for _, s in sensor_dict.items():
            bp = blueprints.find(s.blueprint)

            for key, val in s.attributes.items():
                bp.set_attribute(key, str(val))

            s_actor = world.spawn_actor(bp, s.transform, target_actor)
            s_actor.listen(s.callback)

            pool.append(s_actor)

        yield
    finally:
        # Need to wait for the sensor callbacks to catch up.
        time.sleep(10)

        for s_actor in pool:
            s_actor.destroy()

        pool.clear()


def make_sensor_dict(*yaml_paths):
    import yaml
    import pathlib

    sensors = dict()

    for path in map(pathlib.Path, yaml_paths):
        config = yaml.safe_load(path.read_text())
        sensors[path.stem] = MAPPING[config['sensor']].from_json(config)

    return sensors
