from .renderer import RenderFunction
from . import color
import moderngl
import numpy as np

__all__ = ["MapOutline", "MapRoad"]


def _xy(p):
    # Helper to map carla to numpy
    return np.array((p.x, p.y))


# @RenderFunction.register
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

    def _update_geometry(self, world_map, frame, road_precision=1):
        if self._position is not None:
            return set()

        # Center location and right vector
        position, right = [], []

        for p in world_map.generate_waypoints(1e10):
            # Find all points on the lane
            lane_wps = list(reversed(p.previous_until_lane_start(road_precision)))
            lane_wps.append(p)
            lane_wps.extend(p.next_until_lane_end(road_precision))

            # Add the current lane
            position.extend([_xy(w.transform.location) for w in lane_wps])
            right.extend([_xy(0.5 * w.lane_width * w.transform.get_right_vector()) for w in lane_wps])

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
