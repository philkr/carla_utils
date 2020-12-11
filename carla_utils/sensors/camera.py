import queue
import numpy as np
import carla

from PIL import Image


class Camera(queue.Queue):
    def __init__(self, blueprints, w, h, fov, x, y, z, pitch, yaw):
        super().__init__()

        bp = blueprints.find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(w))
        bp.set_attribute('image_size_y', str(h))
        bp.set_attribute('fov', str(fov))

        loc = carla.Location(x=x, y=y, z=z)
        rot = carla.Rotation(pitch=pitch, yaw=yaw)
        transform = carla.Transform(loc, rot)

        self.blueprint = bp
        self.transform = transform
        self.callback = self.put

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
