from .renderer import RenderFunction
from . import color
from .shaders import RECT_GS, NGON_OUTLINE_GS, FILLED_RECT_GS
import numpy as np
from itertools import cycle

__all__ = ["LaneRenderer"]

def _xy(p):
    # Helper to map carla to numpy
    return np.array((p.x, p.y))

@RenderFunction.register
class LaneRenderer(RenderFunction):
    LANE_COLOR = color.white
    MIN_SIZE = 0.1

    geometry_shader = FILLED_RECT_GS
    uniforms = dict(zorder=0, fill_color=color.white, border_size=0.5)

    def get_all_waypoints(self, world_map):
        middle_marking = world_map.generate_waypoints(2)
        all_wps = []
        for waypoint in middle_marking:
            middle_waypoint = waypoint
            left = []
            while (hasattr(waypoint, 'left_lane_marking')):
                waypoint = waypoint.left_lane_marking
                left.append(waypoint)
            right = []
            # set waypoint back to middle waypoint
            waypoint = middle_waypoint
            while (hasattr(waypoint, 'right_lane_marking')):
                waypoint = waypoint.right_lane_marking
                right.append(waypoint)

            lane_wps = list(reversed(left)) + [middle_waypoint] + list(right)
            all_wps.append(lane_wps)
        return all_wps


    def _update_geometry(self, world_map, frame):
        wps = self.get_all_waypoints(world_map)
        wps_iterator = cycle(wps)


lane = LaneRenderer()
lane.get_waypoints()