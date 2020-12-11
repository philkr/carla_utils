from ctypes import Structure, c_uint8, c_uint16, c_int16, c_uint32, c_int32, c_uint64, c_float, c_double, sizeof
import io
import numpy as np
from copy import copy

__all__ = ['parse', 'Actor', 'Frame']


class Read:
    def __init__(self, f):
        self._f = f

    def _r(self, t):
        return t.from_buffer_copy(self._f.read(sizeof(t))).value

    def bool(self):
        return bool(self._r(c_uint8))

    def uint8(self):
        return self._r(c_uint8)

    def uint16(self):
        return self._r(c_uint16)

    def uint32(self):
        return self._r(c_uint32)

    def uint64(self):
        return self._r(c_uint64)

    def int16(self):
        return self._r(c_int16)

    def int32(self):
        return self._r(c_int32)

    def float32(self):
        return self._r(c_float)

    def float32x3(self):
        return np.array([self._r(c_float), self._r(c_float), self._r(c_float)], dtype='f4')

    def float64(self):
        return self._r(c_double)

    def string(self):
        ln = self._r(c_uint16)
        return self._f.read(ln)

    def read(self, *args, **kwargs):
        return self._f.read(*args, **kwargs)


class Actor:
    def __init__(self, tpe, location, rotation, desc_uid, desc_id, attributes):
        self.type, self.location, self.rotation, self.uid, self.desc, self.attributes = tpe, location, rotation, desc_uid, desc_id, attributes

    @property
    def forward(self):
        sp, sy = np.sin(np.radians(self.rotation[1:]))
        cp, cy = np.cos(np.radians(self.rotation[1:]))
        return np.array([cy * cp, sy * cp, sp], dtype='f4')

    @property
    def right(self):
        sr, sp, sy = np.sin(np.radians(self.rotation))
        cr, cp, cy = np.cos(np.radians(self.rotation))
        return np.array([cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr], dtype='f4')


class Frame:
    def __init__(self, frame_id, duration, elapsed, prev_frame=None):
        self.frame_id = frame_id
        self.duration = duration
        self.elapsed = elapsed
        self._actors = {}
        if prev_frame is not None:
            # We purposely do not deepcopy here
            self._actors = copy(prev_frame._actors)

    @property
    def actors(self):
        return list(self._actors.values())

    def add_actor(self, id, a):
        assert id not in self._actors, 'Actor {} already exists'.format(id)
        self._actors[id] = a

    def delete_actor(self, id):
        del self._actors[id]

    def update_actor(self, id, **kwargs):
        assert id in self._actors, 'Actor {} does not exists'.format(id)
        a = copy(self._actors[id])
        setattr(a, 'id', id)
        for k, v in kwargs.items():
            setattr(a, k, v)
        self._actors[id] = a

    def actor_by_id(self, id):
        if id in self._actors:
            return self._actors[id]
        return None

    @property
    def other_actors(self):
        return [a for a in self.actors if a.type == 0]

    @property
    def vehicles(self):
        return [a for a in self.actors if a.type == 1]

    @property
    def cars(self):
        return [a for a in self.vehicles if 'number_of_wheels' in a.attributes and a.attributes['number_of_wheels'] == 4]

    @property
    def bikes(self):
        return [a for a in self.vehicles if 'number_of_wheels' in a.attributes and a.attributes['number_of_wheels'] == 2]

    @property
    def walkers(self):
        return [a for a in self.actors if a.type == 2]

    @property
    def traffic_lights(self):
        return [a for a in self.actors if a.type == 3]

    @property
    def invalid_actors(self):
        return [a for a in self.actors if a.type == 4]


attribute_type_map = {0: bool, 1: int, 2: float, 3: bytes, 4: lambda s: [int(c) for c in s.split(b',')]}


def parse(filename):
    frames = []
    world_map = None

    with open(filename, 'rb') as f:
        r = Read(f)
        version, magic = r.uint16(), r.string()
        timestamp = r.uint32(), r.uint32()
        map_name = r.string()

        current_frame = None
        while f.readable():
            try:
                pid, sz = r.uint8(), r.uint32()
            except ValueError:
                break
            packet_data = r.read(sz)
            r_packet = Read(io.BytesIO(packet_data))

            if pid == 0:  # Frame start
                current_frame = Frame(r_packet.uint64(), r_packet.float64(), r_packet.float64(), current_frame)

            elif pid == 1:  # Frame end
                frames.append(current_frame)

            elif pid == 2:  # Event Add
                try:
                    n = r_packet.uint16()
                except ValueError:
                    n = 0
                for _ in range(n):
                    id_, type_ = r_packet.uint32(), r_packet.uint8()
                    loc, rot = r_packet.float32x3()/100., r_packet.float32x3()
                    desc_uid, desc_str = r_packet.uint32(), r_packet.string()
                    attributes = {}
                    na = r_packet.uint16()
                    for _ in range(na):
                        tpe = attribute_type_map[r_packet.uint8()]
                        name = r_packet.string().decode()
                        attributes[name] = tpe(r_packet.string())
                    current_frame.add_actor(id_, Actor(type_, loc, rot, desc_uid, desc_str, attributes))

            elif pid == 3:  # Event Del
                n = r_packet.uint16()
                for _ in range(n):
                    id_ = r_packet.uint32()
                    current_frame.delete_actor(id_)

            elif pid == 4:  # Event Parent
                n = r_packet.uint16()
                if n:
                    # Ignore
                    pass

            elif pid == 5:  # Event Collision
                n = r_packet.uint16()
                if n:
                    # Ignore
                    pass

            elif pid == 6:  # Event Position
                n = r_packet.uint16()
                for _ in range(n):
                    id_ = r_packet.uint32()
                    loc = r_packet.float32x3()/100.
                    current_frame.update_actor(id_, location=loc, rotation=r_packet.float32x3())

            elif pid == 7:  # TrafficLight
                n = r_packet.uint16()
                for _ in range(n):
                    id_ = r_packet.uint32()
                    current_frame.update_actor(id_, frozen=r_packet.bool(), elapsed=r_packet.float32(),
                                               state=r_packet.uint8())

            elif pid == 8:  # Vehicle animation
                n = r_packet.uint16()
                for _ in range(n):
                    id_ = r_packet.uint32()
                    current_frame.update_actor(id_, steering=r_packet.float32(), throttle=r_packet.float32(),
                                               brake=r_packet.float32(), handbrake=r_packet.bool(),
                                               gear=r_packet.int32())

            elif pid == 9:  # Walker animation
                n = r_packet.uint16()
                if n:
                    # Ignore
                    pass

            elif pid == 12: # Dynamic actor kinematics
                n = r_packet.uint16()
                for _ in range(n):
                    id_ = r_packet.uint32()
                    current_frame.update_actor(id_, linear_velocity=r_packet.float32x3(),
                                               angular_velocity=r_packet.float32x3())

            elif pid == 250:  # Our custom map encoding
                # Otherwise just compile the carla map code
                from .. import map
                from zlib import decompress
                map_name = r_packet.string()
                map_data = decompress(r_packet.read())
                world_map = map.Map(map_name, map_data)

    return world_map, frames
