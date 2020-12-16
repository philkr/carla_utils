import queue
import numpy as np

from PIL import Image
from carla import Location, Rotation, Transform


class Camera(queue.Queue):
    blueprint = 'sensor.camera.rgb'

    def __init__(self, attributes, x=0.0, y=0.0, z=0.0, pitch=0.0, yaw=0.0, roll=0.0):
        super().__init__()

        self.attributes = attributes
        self.transform = Transform(Location(x=x, y=y, z=z), Rotation(pitch=pitch, yaw=yaw))
        self.callback = self.put

    @classmethod
    def from_json(cls, config):
        return cls(config['attributes'], **config['transform'])

    def __iter__(self):
        last_data = None

        while self.qsize() > 0:
            data = self.get()

            if last_data is None or data.timestamp != last_data.timestamp:
                yield data

            last_data = data

    def save(self, data, save_dir, frame_id):
        array = np.frombuffer(data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (data.height, data.width, 4))
        array = array[..., :3]
        array = array[..., ::-1]

        Image.fromarray(array).save(save_dir / ('%05d.png' % frame_id))
