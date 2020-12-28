import logging
import random
import queue

from contextlib import contextmanager
from pathlib import Path
from typing import List
from carla_utils.recording.config import Configuration, Required, Settings


@Configuration.register('render')
class RenderSettings(Settings):
    weather: str = None


def weather_presets():
    import carla
    import re
    return [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]


@Configuration.register('sensor', repeated=True)
class SensorSettings(Settings):
    class Transform(Settings):
        location: List[float]  # x, y, z
        roll: float = 0
        pitch: float = 0
        yaw: float = 0

        def to_carla(self):
            import carla
            location = carla.Location(*self.location)
            rotation = carla.Rotation(pitch=self.pitch, yaw=self.yaw, roll=self.roll)
            return carla.Transform(location, rotation)

    # Sensor settings
    name: str = Required
    type: str = Required
    output_format: str = None
    output_attributes: dict = {}

    # Carla sensor settings
    target_actor: int = None
    transform: Transform = None
    attributes: dict = {}


class SensorRegistry:
    _all = {}

    @staticmethod
    def register(cls):
        assert hasattr(cls, 'blueprint') and cls.blueprint is not None,\
            'Blueprint not defined for {!r}'.format(cls.__name__)
        # Note to future self: I decided to keep indexing by name here to allow multiple sensors with the same blueprint
        SensorRegistry._all[cls.__name__] = cls
        return cls

    @staticmethod
    def find(name):
        return SensorRegistry._all.get(name)


class SensorSyncWorld:
    def __init__(self, world):
        self._world = world
        self._queue = queue.Queue()
        self._sensors = {}

    def register_sensor(self, name, obj):
        self._sensors[name] = obj

    def tick(self, *args, timeout=1.0, **kwargs):
        world_tick = self._world.tick(*args, **kwargs)
        unfinished = set(self._sensors.keys())

        try:
            while unfinished:
                tick, name = self._queue.get(timeout=timeout)
                if tick == world_tick:
                    unfinished.remove(name)
                else:
                    logging.warn('World on tick {!r}, Sensor {!r} on tick {:d}.'.format(world_tick, name, tick))
        except queue.Empty:
            logging.warn('Not all sensors finished on tick {:d}, Unfinished: {!r}.'.format(world_tick, unfinished))

        return world_tick

    def _sensor_tick(self, frame_number, name):
        self._queue.put((frame_number, name))

    def __getattr__(self, item):
        return self.__dict__[item] if item[0] == '_' else getattr(self._world, item)

    # Allow SensorSyncWorld to act as a dict of sensors
    def __getitem__(self, name):
        return self._sensors.get(name, None)

    def __iter__(self):
        yield from iter(self._sensors)

    def __len__(self):
        return len(self._sensors)

    def items(self):
        yield from self._sensors.items()

    def values(self):
        yield from self._sensors.values()

    def keys(self):
        yield from self._sensors.keys()


class Sensor(SensorRegistry):
    blueprint = None

    def __init__(self, world: SensorSyncWorld, settings: SensorSettings, output_path: Path):
        # Create the blueprint
        blueprint = world.get_blueprint_library().find(self.blueprint)
        assert blueprint is not None, 'Sensor blueprint {!r} not found'.format(self.blueprint)
        for key, val in settings.attributes.items():
            blueprint.set_attribute(key, str(val))

        # Create the transform
        transform = settings.transform.to_carla()

        # Find the target actor if needed and spawn a new actor
        if settings.target_actor is not None:
            target_actor = world.get_actor(settings.target_actor)
            assert target_actor is not None, 'Invalid target_actor {:d}'.format(settings.target_actor)
            self.actor = world.spawn_actor(blueprint, transform, target_actor)
        else:
            self.actor = world.spawn_actor(blueprint, transform)

        # and setup the callback
        world.register_sensor(settings.name, self)

        self.finalize = lambda frame_number: world._sensor_tick(frame_number, settings.name)
        self.actor.listen(self.callback)

    def _callback(self, x):
        pass

    def callback(self, x):
        self._callback(x)
        self.finalize(x.frame_number)

    def cleanup(self):
        if hasattr(self, 'actor') and self.actor is not None:
            self.actor.destroy()
            self.actor = None

    def __del__(self):
        self.cleanup()


@contextmanager
def sensors(world, render_config: RenderSettings, sensor_config: List[SensorSettings], output_path: Path):
    import carla
    # Render settings
    if render_config.weather is None or str(render_config.weather).lower() == 'none':
        world.set_weather(getattr(carla.WeatherParameters, random.choice(weather_presets())))
    else:
        assert hasattr(carla.WeatherParameters, render_config.weather),\
            'Invalid weather "{}"'.format(render_config.weather)
        world.set_weather(getattr(carla.WeatherParameters, render_config.weather))

    # Render things as long as we have sensors
    settings = world.get_settings()
    settings.no_rendering_mode = len(sensor_config) < 1
    world.apply_settings(settings)

    # Make world.tick wait for sensors...
    world = SensorSyncWorld(world)

    # Setup all sensors
    sensors = []
    for c in sensor_config:
        S = Sensor.find(c.type)
        assert S is not None, 'No sensor of type {!r} found!'.format(c.type)
        sensors.append(S(world, c, output_path))

    try:
        # leave the context manager
        yield world
    finally:
        # cleanup
        for s in sensors:
            s.cleanup()
