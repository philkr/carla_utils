import numpy as np
import logging

from pathlib import Path
from .sensor import Sensor, SensorSettings


IMAGE_FORMATS = {'jpg', 'png', 'webp', 'bmp'}
VIDEO_FORMATS = {'mov', 'avi', 'mpg', 'mpeg', 'mp4', 'mkv', 'wmv'}


class TickingImageWriter:
    def __init__(self, path_pattern: str, **kwargs):
        self.tick = 0
        self.path_pattern = path_pattern
        self.kwargs = kwargs

    def close(self):
        pass

    def append_data(self, data, meta={}):
        import imageio
        writer = imageio.get_writer(self.path_pattern.format(self.tick), **self.kwargs)
        writer.append_data(data)
        writer.close()
        self.tick += 1


class TickingPILWriter:
    def __init__(self, path_pattern: str, **kwargs):
        self.tick = 0
        self.path_pattern = path_pattern
        self.kwargs = kwargs

    def close(self):
        pass

    def append_data(self, data, meta={}):
        from PIL import Image

        Image.fromarray(data).save(self.path_pattern.format(self.tick))
        self.tick += 1


@Sensor.register
class RGBCamera(Sensor):
    blueprint = 'sensor.camera.rgb'

    def __init__(self, world, settings: SensorSettings, output_path: Path):
        super().__init__(world, settings, output_path)

        import imageio

        self.writer = None

        if settings.output_format == 'PIL':
            sensor_output_path = output_path / settings.name
            sensor_output_path.mkdir(exist_ok=False, parents=False)
            output_file = str(sensor_output_path / '{:06d}.png')

            self.writer = TickingPILWriter(output_file, **settings.output_attributes)
        elif settings.output_format in IMAGE_FORMATS:
            output_file = str(output_path) + settings.name + '_{:06d}.' + settings.output_format
            self.writer = TickingImageWriter(output_file, **settings.output_attributes)
        elif settings.output_format in VIDEO_FORMATS:
            output_file = str(output_path) + settings.name + '.' + settings.output_format
            self.writer = imageio.get_writer(output_file, **settings.output_attributes)
        elif settings.output_format is not None:
            logging.warning('Sensor {!r}: Unknown output_format {!r}'.format(settings.name, settings.output_format))
        else:
            logging.warning('Sensor {!r}: no output_format specified.'.format(settings.name))

        f = settings.attributes['image_size_x'] / (2 * np.tan(settings.attributes['fov'] * np.pi / 360))
        cx = settings.attributes['image_size_x'] / 2
        cy = settings.attributes['image_size_y'] / 2

        self.metadata['intrinsics'] = [
            [f, 0, cx],
            [0, f, cy],
            [0, 0, 1]]

        P = np.float32([
            [ 0,  0, 1, 0],
            [-1,  0, 0, 0],
            [ 0, -1, 0, 0],
            [ 0,  0, 0, 1],
        ])

        self.metadata['transform'] = (np.float32(self.metadata['transform']) @ P).tolist()

    def _callback(self, sensor_data):
        super()._callback(sensor_data)

        array = np.frombuffer(sensor_data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (sensor_data.height, sensor_data.width, 4))
        array = array[..., :3]
        array = array[..., ::-1]

        # TODO: Push this off to ray at some point if it's faster
        if self.writer is not None:
            self.writer.append_data(array)
