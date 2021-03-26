import pathlib
import numpy as np

from .sensor import SensorSyncWorld, SensorRegistry, SensorSettings
from .utils import transform_to_matrix


def get_wheel_base(actor):
    wheels = actor.get_physics_control().wheels
    across = wheels[0].position - wheels[2].position

    return np.linalg.norm([across.x, across.y, across.z]) / 100.0


def get_matrix(transform, is_cam):
    """
    Creates np.array from carla transform.
    """

    rotation = transform.rotation
    location = transform.location
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))
    matrix = np.eye(4, dtype=np.float32)
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r

    # go from (x-front, y-right, z-up) to (x-front, y-left, z-up)
    matrix[:, 1] *= -1
    matrix[1, :] *= -1

    return matrix


class Vehicle(object):
    def __init__(self, actor):
        self.actor = actor
        self.wheel_base = get_wheel_base(self.actor)

    def state_dict(self):
        control = self.actor.get_control()
        velocity = self.actor.get_velocity()
        transform = self.actor.get_transform()

        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])

        steer = control.steer
        throttle = control.throttle
        curvature = -np.tan(steer) / self.wheel_base
        pose = transform_to_matrix(transform).tolist()
        yaw = transform.rotation.yaw

        return {
            'steer': steer,
            'throttle': throttle,
            'curvature': curvature,
            'speed': speed,
            'pose': pose,
            'yaw': yaw,
        }


@SensorRegistry.register
class GPSSensor(SensorRegistry):
    blueprint = 'hack'

    def __init__(self, world: SensorSyncWorld, settings: SensorSettings, output_path: pathlib.Path):
        world.register_sensor(settings.name, self)
        world.on_tick(self.callback)

        self.finalize = lambda frame_number: world._sensor_tick(frame_number, settings.name)

        self.writer = (output_path / ('%s.txt' % settings.name)).open('w')
        self.vehicle = Vehicle(world.get_actor(settings.target_actor))

        self.metadata = {'wheel_base': self.vehicle.wheel_base}

    def callback(self, snapshot):
        if self.vehicle is not None and self.writer is not None:
            self.writer.write(str(self.vehicle.state_dict())+'\n')

        self.finalize(snapshot.frame)

    def cleanup(self):
        self.writer.close()
        self.writer = None

        self.vehicle = None

    def __del__(self):
        self.cleanup()
