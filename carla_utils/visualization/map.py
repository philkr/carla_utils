from .renderer import RenderFunction
from . import color
import moderngl
import numpy as np

__all__ = ["MapOutline", "MapRoad"]


def _xy(p):
    # Helper to map carla to numpy
    return np.array((p.x, p.y))


def _get_lanes(world_map, road_precision=1, filter=None):
    for p in world_map.generate_waypoints(1e10):
        # Find all points on the lane
        lane_wps = list(reversed(p.previous_until_lane_start(road_precision)))
        lane_wps.append(p)
        lane_wps.extend(p.next_until_lane_end(road_precision))
        filtered_lane = filter(lane_wps) if filter is not None else lane_wps

        if filtered_lane is not None and len(filtered_lane) is not None:
            yield filtered_lane


@RenderFunction.register
class MapOutline(RenderFunction):
    render_type = moderngl.LINE_STRIP
    geometry_shader = """{{HEAD}}
    layout(lines) in;
    layout(line_strip, max_vertices = 4) out;
    uniform vec3 fill_color = vec3(0,0,0);
    void main(){
        gs_out.color = fill_color;
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

    uniforms = dict(fill_color=color.NP['aluminium4'], zorder=-11)

    def _update_geometry(self, world_map, frame, road_precision=1, filter=None):
        if 'position' in self._bo:
            return {}

        # Center location and right vector
        position, right = [], []

        for lane_wps in _get_lanes(world_map, road_precision, filter):
            # Add the current lane
            position.extend([_xy(w.transform.location) for w in lane_wps])
            right.extend([_xy(0.5 * w.lane_width * w.transform.get_right_vector()) for w in lane_wps])

            # and a end-primitive marker (right=0)
            position.append((0, 0))
            right.append((0, 0))

        return {'position': np.array(position, dtype='f4'), 'right': np.array(right, dtype='f4')}


@RenderFunction.register
class MapRoad(MapOutline):
    geometry_shader = """{{HEAD}}
    layout(lines) in;
    layout(triangle_strip, max_vertices = 4) out;
    uniform vec3 fill_color = vec3(0,0,0);
    void main(){
        gs_out.color = fill_color;
        if (length(gs_in[0].right) > 1e-1 && length(gs_in[1].right) > 1e-1) {
            for (int i=0; i<2; i++)
                for (int r=-1; r<=1; r+=2) {
                    gl_Position = vec4(view_matrix * vec3(gs_in[i].position+float(r)*gs_in[i].right, 1), -zorder/100., 1);
                    EmitVertex();
                }
            EndPrimitive();
        }
    }"""
    uniforms = dict(fill_color=color.NP['aluminium1'], zorder=-10)


@RenderFunction.register
class LaneDirectionRenderer(MapOutline):
    render_type = moderngl.POINTS
    geometry_shader = """{{HEAD}}
    layout(points) in;
    layout(triangle_strip, max_vertices = 3) out;
    uniform vec3 fill_color = vec3(0,0,0);
    uniform float scale = 1;
    void main(){
        if (length(gs_in[0].right) > 1e-3 && length(gs_in[0].position) > 1e-3) {
            vec2 right = 0.5*scale*gs_in[0].right, forward = scale*vec2(gs_in[0].right.y, -gs_in[0].right.x);

            gs_out.color = fill_color;
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position + right  , 1), -zorder/100., 1); EmitVertex();
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position + forward, 1), -zorder/100., 1); EmitVertex();
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position - right  , 1), -zorder/100., 1); EmitVertex();
            EndPrimitive();
        }
    }"""

    uniforms = dict(zorder=-8, fill_color=color.NP['aluminium2'], scale=0.5)

    def _update_geometry(self, world_map, frame, road_precision=3.0):
        return super()._update_geometry(world_map, frame, road_precision,
                                        filter=lambda l: [i for i in l if not i.is_junction])


# @RenderFunction.register
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
        for lane_wps in _get_lanes(world_map, road_precision):
            colors.extend([(w.s, 0.0, 0.0) for w in lane_wps])
            colors.append((0, 0, 0))
        self._color = np.array(colors, dtype='f4')

        return super()._update_geometry(world_map, frame, road_precision)
