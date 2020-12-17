import queue

from contextlib import contextmanager
from .sensors import BLUEPRINT_TO_SENSOR
from .parse import parse


class SensorConfiguration(object):
    def __init__(self, sensors, save_dir=None):
        self.save_dir = save_dir

        self.job_queue = queue.Queue()
        self.sensors = sensors
        self.actors = dict()

    def hook(self, world):
        for s in self.sensors:
            self.actors[s.name] = s.hook(world, self.job_queue)

    def wait(self, frame_number, timeout=1.0):
        unfinished = {s.name for s in self.sensors}

        while unfinished:
            name, frame = self.job_queue.get(True, timeout=timeout)

            if name in unfinished and frame == frame_number:
                unfinished.remove(name)

    def override(self, override_args):
        override_dict = dict()

        for i in range(0, len(override_args), 2):
            key = override_args[i].lstrip('-')
            val = eval(override_args[i+1])

            override_dict[key] = val

        for s in self.sensors:
            s.override(override_dict)

    @classmethod
    def from_files(cls, config_yaml, save_dir):
        import yaml

        assert save_dir.parent.exists(), '%s is not a valid directory.' % save_dir
        assert not save_dir.exists(), '%s already exists.' % save_dir

        save_dir.mkdir()

        sensors = list()

        for config in yaml.load_all(config_yaml.read_text()):
            name = config['name']
            sensor_dir = save_dir / name
            sensor_dir.mkdir()

            sensor_class = BLUEPRINT_TO_SENSOR[config['sensor']]
            s = sensor_class.from_json(name, config, save_dir=sensor_dir)
            sensors.append(s)

        return cls(sensors)

    def __del__(self):
        for _, a in self.actors.items():
            a.destroy()

        self.actors.clear()


@contextmanager
def replay(recording, host, port, fps):
    import carla

    # Skip the last second. CARLA hangs if we run to the end...
    world_map, frames = parse(recording)
    ticks = len(frames) - fps

    client = carla.Client(host, port, worker_threads=4)
    client.set_replayer_time_factor(1.0)

    world = client.load_world(world_map.name)

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = 1.0 / fps
        world.apply_settings(settings)

        print(client.replay_file(recording, 0.0, ticks / fps, 0))

        # Tick once to start the recording.
        world.tick()

        yield world, ticks
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
