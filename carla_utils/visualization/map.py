from .renderer import RenderFunction
from . import color
import moderngl
import numpy as np

__all__ = ["MapOutline", "MapRoad"]


def _xy(p):
    # Helper to map carla to numpy
    return np.array((p.x, p.y))


@RenderFunction.register
class MapOutline(RenderFunction):
    render_type = moderngl.LINE_STRIP
    geometry_shader = """{{HEAD}}
    layout(lines) in;
    layout(line_strip, max_vertices = 4) out;
    void main(){
        gs_out.color = gs_in[0].color;
        if (length(gs_in[0].right) > 1e-1 && length(gs_in[1].right) > 1e-1) {
            for (int r=-1; r<=1; r+=2) {
                for (int i=0; i<2; i++) {
                    gl_Position = vec4(view_matrix * vec3(gs_in[i].position+float(r)*gs_in[i].right, 1), -zorder/100., 1);
                    EmitVertex();
                }
                EndPrimitive();
            }
        }
    }"""

    uniforms = dict(const_color=color.NP4['aluminium4'], zorder=-11)
    LANE_WIDTH = 0.5

    def _get_lanes(self, world_map, road_precision):
        for p in world_map.generate_waypoints(1e10):
            # Find all points on the lane
            lane_wps = list(reversed(p.previous_until_lane_start(road_precision)))
            lane_wps.append(p)
            lane_wps.extend(p.next_until_lane_end(road_precision))

            yield lane_wps

    def _update_geometry(self, world_map, frame, road_precision=1):
        if self._position is not None:
            return set()

        # Center location and right vector
        position, right = [], []

        for lane_wps in self._get_lanes(world_map, road_precision):
            # Add the current lane
            position.extend([_xy(w.transform.location) for w in lane_wps])
            right.extend([_xy(self.LANE_WIDTH * w.lane_width * w.transform.get_right_vector()) for w in lane_wps])

            # and a end-primitive marker (right=0)
            position.append((0, 0))
            right.append((0, 0))

        self._position, self._right = np.array(position, dtype='f4'), np.array(right, dtype='f4')

        return set()


@RenderFunction.register
class MapRoad(MapOutline):
    geometry_shader = """{{HEAD}}
    layout(lines) in;
    layout(triangle_strip, max_vertices = 4) out;
    void main(){
        gs_out.color = gs_in[0].color;
        if (length(gs_in[0].right) > 1e-1 && length(gs_in[1].right) > 1e-1) {
            for (int i=0; i<2; i++)
                for (int r=-1; r<=1; r+=2) {
                    gl_Position = vec4(view_matrix * vec3(gs_in[i].position+float(r)*gs_in[i].right, 1), -zorder/100., 1);
                    EmitVertex();
                }
            EndPrimitive();
        }
    }"""
    uniforms = dict(const_color=color.NP4['aluminium1'], zorder=-10)


@RenderFunction.register
class CenterLaneRenderer(MapOutline):
    geometry_shader = """{{HEAD}}
    layout(lines) in;
    layout(triangle_strip, max_vertices = 4) out;
    uniform vec3 lane_color = vec3(0,0,0);
    uniform vec3 nolane_color = vec3(0,0,0);
    void main(){
        if (length(gs_in[0].right) > 1e-3 && length(gs_in[1].right) > 1e-3) {
            for (int i=0; i<2; i++)
                for (int r=-1; r<=1; r+=2) {
                    gl_Position = vec4(view_matrix * vec3(gs_in[i].position+float(r)*gs_in[i].right, 1), -zorder/100., 1);

                    float dist = mod(gs_in[i].color[0], 3.0);
                    float alpha = float(dist <= 1.0);
                    gs_out.color = alpha * lane_color + (1.0 - alpha) * nolane_color;

                    EmitVertex();
                }
            EndPrimitive();
        }
    }"""

    LANE_WIDTH = 0.02
    uniforms = dict(zorder=-9, lane_color=color.NP['aluminium4'], nolane_color=color.NP['aluminium1'])

    def _update_geometry(self, world_map, frame, road_precision=0.2):
        if self._position is not None:
            return super()._update_geometry(world_map, frame, road_precision)

        colors = []
        for lane_wps in self._get_lanes(world_map, road_precision):
            colors.extend([(w.s, 0.0, 0.0) for w in lane_wps])
            colors.append((0, 0, 0))
        self._color = np.array(colors, dtype='f4')

        return super()._update_geometry(world_map, frame, road_precision)


@RenderFunction.register
class CenterLaneArrowRenderer(MapOutline):
    render_type = moderngl.POINTS
    geometry_shader = """{{HEAD}}
    layout(points) in;
    layout(triangle_strip, max_vertices = 3) out;
    uniform vec3 fill_color = vec3(0,0,0);
    void main(){
        if (length(gs_in[0].right) > 1e-3 && length(gs_in[0].position) > 1e-3) {
            gs_out.color = fill_color;

            for (int r = -1; r <= 1; r+=1) {
                float f = 1.0 - abs(float(r));

                vec2 right = 0.75 * r * normalize(gs_in[0].right);
                vec2 forward = 0.75 * (1.0 - abs(r)) * normalize(gs_in[0].forward);

                gl_Position = vec4(view_matrix * vec3(gs_in[0].position+right+forward, 1), -zorder/100., 1);
                EmitVertex();
            }

            EndPrimitive();
            gs_out.color = fill_color;
        }
    }"""

    uniforms = dict(zorder=-8, fill_color=color.NP['butter3'])

    def _update_geometry(self, world_map, frame, road_precision=15.0):
        if self._position is not None:
            return super()._update_geometry(world_map, frame, road_precision)

        forward = []
        for lane_wps in self._get_lanes(world_map, road_precision):
            forward.extend([_xy(w.transform.get_forward_vector()) for w in lane_wps])
            forward.append((0, 0))
        self._forward = np.array(forward, dtype='f4')

        return super()._update_geometry(world_map, frame, road_precision)
