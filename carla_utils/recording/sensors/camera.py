import numpy as np
import imageio

from .base import Sensor, transform_from_json


class Camera(Sensor):
    blueprint = 'sensor.camera.rgb'

    def __init__(self, *args, save_dir=None, **kwargs):
        super().__init__(*args, **kwargs)

        self.target_actor = 86
        self.save_dir = save_dir

    @classmethod
    def from_json(cls, config, save_dir=None):
        attributes = config['attributes']
        transform = transform_from_json(config['transform'])

        return cls(attributes, transform, save_dir=save_dir)

    def _locked_callback(self, sensor_data):
        array = np.frombuffer(sensor_data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
        array = array[..., :3]
        array = array[..., ::-1]

        writer = imageio.get_writer(self.save_dir / ('%d.png' % self.ticks))
        writer.append_data(array)
        writer.close()

        super()._locked_callback(sensor_data)
