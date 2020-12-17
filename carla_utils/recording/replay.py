import time

from contextlib import contextmanager
from .sensors import BLUEPRINT_TO_SENSOR
from .parse import parse


class SensorConfiguration(object):
    def __init__(self, sensors, save_dir=None):
        self.save_dir = save_dir

        self.sensors = sensors
        self.actors = dict()

    def hook(self, world):
        for s_name, s in self.sensors.items():
            self.actors[s_name] = s.hook(world)

    def finalize(self, frame_number, sleep_time=1.0, retries=10):
        print('World frame: %d' % frame_number)

        for s_name, a in self.actors.items():
            print('Sensor: %s' % s_name)

            for retry in range(1, retries+1):
                n = self.sensors[s_name].get_frame_number()
                print('Frame: %d try %d / %d' % (n, retry, retries), end='\r')

                if n == frame_number:
                    print('\nSuccess.')
                    break

                time.sleep(sleep_time)
            else:
                print('\nFailed.')

            a.destroy()

        self.actors.clear()

    @classmethod
    def from_files(cls, yaml_files, save_dir):
        import yaml
        import pathlib

        assert save_dir.parent.exists(), '%s is not a valid directory.' % save_dir
        assert not save_dir.exists(), '%s already exists.' % save_dir

        save_dir.mkdir()

        sensors = dict()

        for path in map(pathlib.Path, yaml_files):
            name = path.stem

            sensor_dir = save_dir / name
            sensor_dir.mkdir()

            config = yaml.safe_load(path.read_text())
            sensor_class = BLUEPRINT_TO_SENSOR[config['sensor']]
            sensors[name] = sensor_class.from_json(config, save_dir=sensor_dir)

        return cls(sensors)


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
