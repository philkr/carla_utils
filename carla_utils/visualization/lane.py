from .renderer import RenderFunction
from . import color
import numpy as np
import moderngl

__all__ = ["LaneRenderer"]


def _xy(p):
    # Helper to map carla to numpy
    return np.array((p.x, p.y))


@RenderFunction.register
class LaneRenderer(RenderFunction):
    render_type = moderngl.LINE_STRIP
    geometry_shader = """{{HEAD}}
    layout(lines) in;
    layout(triangle_strip, max_vertices = 4) out;
    uniform vec3 lane_color = vec3(0,0,0);
    void main(){
        if (length(gs_in[0].right) > 1e-2 && length(gs_in[1].right) > 1e-2) {
            for (int i=0; i<2; i++)
                for (int r=-1; r<=1; r+=2) {
                    gl_Position = vec4(view_matrix * vec3(gs_in[i].position+float(r)*gs_in[i].right, 1), -zorder/100., 1);
                    float dist = mod(gs_in[i].color[0], 3.0);
                    float alpha = float(dist >= 1.0);
                    gs_out.color = alpha * lane_color;

                    EmitVertex();
                }
            EndPrimitive();
        }
    }"""

    uniforms = dict(zorder=-9, lane_color=color.butter1)

    def _update_geometry(self, world_map, frame, road_precision=0.1):
        if self._position is not None:
            return set()

        # Center location and right vector
        position, right, colors = [], [], []
        seen = set()

        for p in world_map.generate_waypoints(1e10):
            # Find all points on the lane
            lane_wps = list(reversed(p.previous_until_lane_start(road_precision)))
            lane_wps.append(p)
            lane_wps.extend(p.next_until_lane_end(road_precision))

            # Make sure we don't add duplicate waypoints.
            seen.update([x.id for x in lane_wps])

            points = [_xy(w.transform.location) for w in lane_wps]
            delta = [(0, 0)] + [points[i+1] - points[i] for i in range(len(points)-1)]
            cumulative = np.cumsum(np.linalg.norm(delta, axis=1))

            # Add the current lane
            position.extend(points)
            right.extend([_xy(0.025 * w.lane_width * w.transform.get_right_vector()) for w in lane_wps])
            colors.extend([(c, 0.0, 0.0) for c in cumulative])

            # and a end-primitive marker (right=0)
            position.append((0, 0))
            right.append((0, 0))
            colors.append((1, 1, 1))

        self._position = np.array(position, dtype='f4')
        self._right = np.array(right, dtype='f4')
        self._color = np.array(colors, dtype='f4')

        return set()
