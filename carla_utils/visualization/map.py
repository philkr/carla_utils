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

    uniforms = dict(fill_color=color.aluminium4, zorder=-11)

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
    uniforms = dict(fill_color=color.aluminium1, zorder=-10)


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

    uniforms = dict(zorder=-8, fill_color=color.aluminium2, scale=0.5)

    def _update_geometry(self, world_map, frame, road_precision=3.0):
        return super()._update_geometry(world_map, frame, road_precision,
                                        filter=lambda l: [i for i in l if not i.is_junction])


lane_marking_textures = np.array([
    [  # Other
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ],
    [  # Broken
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ],
    [  # Solid
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
    ],
    [  # SolidSolid
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
    ],
    [  # SolidBroken
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 0],
        [1, 0, 0],
        [1, 0, 0],
    ],
    [  # BrokenSolid
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
        [0, 0, 1],
    ],
    [  # BrokenBroken
        [1, 0, 1],
        [1, 0, 1],
        [1, 0, 1],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ],
    [  # BottsDots
        [0, 1, 0],
        [0, 0, 0],
        [0, 1, 0],
        [0, 0, 0],
        [0, 1, 0],
        [0, 0, 0],
    ],
    [  # Grass
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
    ],
    [  # Curb
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
        [0, 1, 0],
    ],
    [  # None
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
    ],
])

lane_marking_color = [
    color.white, #Standard
    color.skyblue1, #Blue
    color.chameleon1, #Green
    color.scarletred1, #Red
    color.white, #White
    color.butter1, #Yellow
    color.plum1, #Other
]


@RenderFunction.register
class CenterLaneRenderer(MapOutline):
    geometry_shader = """{{HEAD}}

flat out int marking_id;
out vec2 tex_coord;

layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

uniform float scale=1;

void main(){
    if (length(gs_in[0].right) > 1e-3 && length(gs_in[1].right) > 1e-3) {
        for (int i=0; i<2; i++) {
            marking_id = int(gs_in[i].marking);
            gs_out.color = gs_in[i].color;
            
            tex_coord = vec2(gs_in[i].s / scale, 1);
            gl_Position = vec4(view_matrix * vec3(gs_in[i].position+gs_in[i].right, 1), -zorder/100., 1);
            EmitVertex();
            
            tex_coord = vec2(gs_in[i].s / scale, 0);
            gl_Position = vec4(view_matrix * vec3(gs_in[i].position-gs_in[i].right, 1), -zorder/100., 1);
            EmitVertex();
        }
        EndPrimitive();
    }
}"""
    fragment_shader = """
#version 330
in GS_OUT {
    vec3 color;
} fs_in;
flat in int marking_id;
in vec2 tex_coord;

out vec4 fragColor;

uniform sampler2DArray lane_markings;

void main() {
    float lv = texture(lane_markings, vec3(tex_coord.yx, float(marking_id))).x;
    fragColor = vec4(fs_in.color, lv);
}
"""

    uniforms = dict(zorder=-9)

    def _update_geometry(self, world_map, frame, road_precision=1):
        if 'position' in self._bo:
            return {}

        # Side location and right vector
        position, right, s, marking, colors = [], [],  [], [], []

        for lane_wps in _get_lanes(world_map, road_precision):
            for d, side in [(-1, 'left'), (1, 'right')]:
                for w in lane_wps:
                    m = getattr(w, '{}_lane_marking'.format(side))
                    if m.type is not m.type.NONE:
                        x = _xy(w.transform.location)
                        r = 0.5 * d * w.lane_width * _xy(w.transform.get_right_vector())
                        c = lane_marking_color[m.color]

                        position.append(x+r)
                        right.append(m.width*_xy(w.transform.get_right_vector()))
                        s.append(w.s)
                        marking.append(int(m.type))
                        if m.type == m.type.Grass:
                            c = color.chameleon3
                        if m.type == m.type.Curb:
                            c = color.aluminium6
                        colors.append(c)

                # and a end-primitive marker (right=0)
                position.append((0, 0))
                right.append((0, 0))
                s.append(0)
                marking.append(0)
                colors.append((0, 0, 0))
        return {'position': np.array(position, dtype='f4'), 'right': np.array(right, dtype='f4'),
                'color': np.array(colors, dtype='f4'),
                's': np.array(s, dtype='f4').reshape(-1, 1),
                'marking': np.array(marking, dtype='f4').reshape(-1, 1)}

    def _setup_vao(self, ctx, vao):
        data = 255*lane_marking_textures.astype(np.uint8)
        # resize the array to avoid aliasing
        data = np.kron(data, np.ones((1, 10, 10), dtype=np.uint8))
        self._lm_tex = ctx.texture_array([data.shape[2], data.shape[1], data.shape[0]], 1, data=data)
        self._lm_tex.filter == (moderngl.NEAREST, moderngl.NEAREST)
        self._lm_tex.use(vao.program['lane_markings'].value)
        ctx.enable(moderngl.BLEND)
        ctx.blend_func = moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA

