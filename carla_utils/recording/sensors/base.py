from threading import Lock
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
    blueprint = None

    def __init__(self, name, attributes, transform):
        self.name = name
        self.attributes = attributes
        self.transform = transform
        self.target_id = 0

        self.lock = Lock()
        self.ticks = 0
        self.frame_number = -1

    def override(self, override_dict):
        self.target_id = override_dict.get('target_id', self.target_id)

    def get_frame_number(self):
        return self.frame_number

    def make_blueprint(self, world):
        blueprint = world.get_blueprint_library().find(self.blueprint)
        for key, val in self.attributes.items():
            blueprint.set_attribute(key, str(val))
        return blueprint

    def hook(self, world, job_queue):
        blueprint = self.make_blueprint(world)
        target_actor = world.get_actor(self.target_id)

        actor = world.spawn_actor(blueprint, self.transform, target_actor)
        actor.listen(lambda x: self.callback(x, job_queue))

        return actor

    def callback(self, sensor_data, job_queue):
        with self.lock:
            self._locked_callback(sensor_data)

        job_queue.put((self.name, sensor_data.frame_number))

    def _locked_callback(self, sensor_data):
        self.ticks += 1
        self.frame_number = max(self.frame_number, sensor_data.frame_number)
