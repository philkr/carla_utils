from carla_utils.recording.config import Configuration, Required, Settings
from contextlib import contextmanager
import numpy as np
from pathlib import Path
from typing import List
import random


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
        location: np.array  # x, y, z
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
        return SensorRegistry._all[name] if name in SensorRegistry._all else None


class Sensor(SensorRegistry):
    blueprint = None

    def __init__(self, world, settings: SensorSettings, output_path: Path):
        # Create the blueprint
        blueprint = world.get_blueprint_library().find(self.blueprint)
        assert blueprint is not None, 'Sensor blueprint {!r} not found'.format(self.blueprint)
        for key, val in settings.attributes.items():
            blueprint.set_attribute(key, str(val))

        # Create the transform
        transform = settings.transform.to_carla()

        # Find the target actor if needed and spawn a new actor
        if settings.target_actor is not None:
            target_actor = world.get_actor(self.target_actor)
            self.actor = world.spawn_actor(blueprint, transform, target_actor)
        else:
            self.actor = world.spawn_actor(blueprint, transform)
        # and setup the callback
        self.actor.listen(self.callback)

    def callback(self, x):
        pass

    def cleanup(self):
        if self.actor is not None:
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

    # Setup all sensors
    sensors = []
    for c in sensor_config:
        S = Sensor.find(c.type)
        assert S is not None, 'No sensor of type {!r} found!'.format(c.type)
        sensors.append(S(world, c, output_path))

    # leave the context manager
    yield sensors

    # Cleanup
    for s in sensors:
        s.cleanup()
