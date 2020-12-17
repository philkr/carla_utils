from abc import ABC
from multiprocessing import Lock
from carla import Location, Rotation, Transform


def location_from_json(data):
    location_keys = {'x', 'y', 'z'}
    location_dict = {k: data.get(k) for k in location_keys & data.keys()}

    return Location(**location_dict)


def rotation_from_json(data):
    rotation_keys = {'pitch', 'roll', 'yaw'}
    rotation_dict = {k: data.get(k) for k in rotation_keys & data.keys()}

    return Rotation(**rotation_dict)


def transform_from_json(data):
    location = location_from_json(data)
    rotation = rotation_from_json(data)

    return Transform(location, rotation)


class Sensor(object):
    blueprint = 'NOT_SET'

    def __init__(self, attributes, transform):
        self.attributes = attributes
        self.transform = transform

        self.lock = Lock()
        self.ticks = 0
        self.frame_number = -1

    def get_frame_number(self):
        with self.lock:
            return self.frame_number

    def make_blueprint(self, world):
        blueprint = world.get_blueprint_library().find(self.blueprint)
        for key, val in self.attributes.items():
            blueprint.set_attribute(key, str(val))
        return blueprint

    def hook(self, world):
        blueprint = self.make_blueprint(world)
        target_actor = world.get_actor(self.target_actor)

        actor = world.spawn_actor(blueprint, self.transform, target_actor)
        actor.listen(self.callback)

        return actor

    def callback(self, sensor_data):
        with self.lock:
            self._locked_callback(sensor_data)

    def _locked_callback(self, sensor_data):
        self.ticks += 1
        self.frame_number = max(self.frame_number, sensor_data.frame_number)
