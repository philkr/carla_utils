from .renderer import RenderFunction
from . import color
from .shaders import RECT_GS, NGON_OUTLINE_GS
import numpy as np

__all__ = ["StopSignRenderer", "YieldSignRenderer", "SpeedLimitRenderer"]


def _xy(p):
    # Helper to map carla to numpy
    return np.array((p.x, p.y))


class AbstractSignRenderer(RenderFunction):
    SIGN_COLOR = color.scarletred3
    SIZE = 10.0
    SIGN_TYPES = []

    geometry_shader = RECT_GS
    uniforms = dict(zorder=0)

    def _update_geometry(self, world_map, frame):
        if self._position is None:
            actors = []
            for t in self.SIGN_TYPES:
                actors.extend(world_map.get_all_landmarks_of_type(t))
            if len(actors):
                self._position = np.zeros((len(actors), 2), dtype='f4')
                self._right = np.zeros((len(actors), 2), dtype='f4')
                self._forward = np.zeros((len(actors), 2), dtype='f4')
                self._color = np.zeros((len(actors), 3), dtype='f4')
                for i, a in enumerate(actors):
                    t = a.transform
                    w, h = 1. / 2, 1. / 2
                    self._position[i] = _xy(t.location)
                    self._right[i] = _xy(t.get_right_vector()) * self.SIZE * w
                    self._forward[i] = _xy(t.get_forward_vector()) * self.SIZE * h
                    self._color[i] = self.SIGN_COLOR
        return set()


@RenderFunction.register
class StopSignRenderer(AbstractSignRenderer):
    SIGN_COLOR = color.scarletred3
    SIZE = 10.0
    SIGN_TYPES = ['206']

    geometry_shader = NGON_OUTLINE_GS % 6
    uniforms = dict(zorder=1)


@RenderFunction.register
class YieldSignRenderer(AbstractSignRenderer):
    SIGN_COLOR = color.scarletred3
    SIZE = 10.0
    SIGN_TYPES = ['205']

    geometry_shader = NGON_OUTLINE_GS % 3
    uniforms = dict(zorder=1.0)


@RenderFunction.register
class SpeedLimitRenderer(AbstractSignRenderer):
    SIGN_COLOR = color.aluminium4
    SIZE = 6.0
    SIGN_TYPES = ['274']

    uniforms = dict(zorder=0.5)
