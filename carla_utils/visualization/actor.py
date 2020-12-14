from .renderer import RenderFunction
from .shaders import NGON_GS, CAR_GS, BIKE_GS
from ..recording import AgentProperty
from . import color
import numpy as np

__all__ = ["TrafficLightRenderer"]


class ActorRenderer(RenderFunction):
    SIZE = 8.0
    ACTOR_TYPE = None
    COLOR = color.aluminium5

    geometry_shader = NGON_GS % 20
    uniforms = dict(zorder=2)

    def color(self, a):
        return self.COLOR

    def _update_geometry(self, world_map, frame):
        actors = getattr(frame, self.ACTOR_TYPE)
        try:
            w, h = self.SIZE
        except TypeError:
            w, h = self.SIZE, self.SIZE
        if len(actors):
            self._position = np.zeros((len(actors), 2), dtype='f4')
            self._right = np.zeros((len(actors), 2), dtype='f4')
            self._forward = np.zeros((len(actors), 2), dtype='f4')
            self._color = np.zeros((len(actors), 3), dtype='f4')
            for i, a in enumerate(actors):
                w2, h2 = w / 2., h / 2.
                prop = AgentProperty.get(a.desc)
                if prop is not None and 'extent' in prop:
                    h2, w2, _ = prop.extent

                self._position[i] = a.location[:2]
                self._right[i] = a.right[:2] * w2
                self._forward[i] = a.forward[:2] * h2
                self._color[i] = self.color(a)
            return {'position', 'right', 'forward', 'color'}


@RenderFunction.register
class CarRenderer(ActorRenderer):
    SIZE = 3., 5.
    ACTOR_TYPE = 'cars'
    COLOR = color.skyblue3

    geometry_shader = CAR_GS
    uniforms = dict(zorder=3, front_color=color.skyblue1)


@RenderFunction.register
class BikeRenderer(ActorRenderer):
    SIZE = 3., 5.
    ACTOR_TYPE = 'bikes'
    COLOR = color.chocolate3

    geometry_shader = BIKE_GS
    uniforms = dict(zorder=3, front_color=color.chocolate1)


@RenderFunction.register
class TrafficLightRenderer(ActorRenderer):
    SIZE = 8.0
    ACTOR_TYPE = 'traffic_lights'

    COLORS = {
        0: color.scarletred2, # red
        1: color.orange2,  # Yellow
        2: color.chameleon3,  # Green
        3: color.aluminium6,  # Off
        -1: color.plum2,   # Unknown
    }

    def color(self, a):
        if hasattr(a, 'state') and a.state in self.COLORS:
            return self.COLORS[a.state]
        return self.COLORS[-1]

    geometry_shader = NGON_GS % 20
    uniforms = dict(zorder=2)
