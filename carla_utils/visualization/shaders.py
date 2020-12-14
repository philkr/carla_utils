GENERIC_VS = """
#version 330

uniform vec4 const_color = vec4(0., 0., 0., 0.);

in vec2 position;
in vec2 forward;
in vec2 right;
in vec3 color;

out VS_OUT {
    vec2 position;
    vec2 forward;
    vec2 right;
    vec3 color;
} vs_out;

void main() {
    vs_out.position = position;
    vs_out.forward = forward;
    vs_out.right = right;
    vs_out.color = color * (1-const_color.w) + const_color.xyz * const_color.w;
    gl_Position = vec4(position, 0, 1);
}
"""
GS_HEAD = """
#version 330

uniform mat3x2 view_matrix;
uniform float zorder;

in VS_OUT {
    vec2 position;
    vec2 forward;
    vec2 right;
    vec3 color;
} gs_in[];
out GS_OUT {
    vec3 color;
} gs_out;
"""
GENERIC_FS = """
#version 330
in GS_OUT {
    vec3 color;
} fs_in;
out vec4 fragColor;
void main() {
    fragColor = vec4(fs_in.color, 1.0);
}
"""

RECT_GS = """{{HEAD}}
layout(points) in;
layout(triangle_strip, max_vertices = 4) out;
void main(){
    gs_out.color = gs_in[0].color;
    gl_Position = vec4(view_matrix * vec3(gs_in[0].position+gs_in[0].right+gs_in[0].forward, 1), -zorder/100., 1);
    EmitVertex();
    gl_Position = vec4(view_matrix * vec3(gs_in[0].position+gs_in[0].right-gs_in[0].forward, 1), -zorder/100., 1);
    EmitVertex();
    gl_Position = vec4(view_matrix * vec3(gs_in[0].position-gs_in[0].right+gs_in[0].forward, 1), -zorder/100., 1);
    EmitVertex();
    gl_Position = vec4(view_matrix * vec3(gs_in[0].position-gs_in[0].right-gs_in[0].forward, 1), -zorder/100., 1);
    EmitVertex();
    EndPrimitive();
}
"""
NGON_OUTLINE_GS = """{{HEAD}}
#define N %d
layout(points) in;
#if N == 3
layout(triangle_strip, max_vertices = 8) out;
#elif N == 4
layout(triangle_strip, max_vertices = 10) out;
#elif N == 5
layout(triangle_strip, max_vertices = 12) out;
#elif N == 6
layout(triangle_strip, max_vertices = 14) out;
#elif N == 7
layout(triangle_strip, max_vertices = 16) out;
#elif N == 8
layout(triangle_strip, max_vertices = 18) out;
#else
layout(triangle_strip, max_vertices = 100) out;
#endif
void main(){
    gs_out.color = gs_in[0].color;
    const float r1 = 1., r2 = 0.7;
    for(int i=0; i<=N; i++) {
        float a = radians(360.*i/N);
        float sa = sin(a), ca = cos(a);
        gl_Position = vec4(view_matrix * vec3(gs_in[0].position+r1*sa*gs_in[0].right-r1*ca*gs_in[0].forward, 1), -zorder/100., 1);
        EmitVertex();
        gl_Position = vec4(view_matrix * vec3(gs_in[0].position+r2*sa*gs_in[0].right-r2*ca*gs_in[0].forward, 1), -zorder/100., 1);
        EmitVertex();
    }
    EndPrimitive();
}
"""
NGON_GS = """{{HEAD}}
#define N %d
layout(points) in;
layout(triangle_strip, max_vertices = N) out;
void main(){
    gs_out.color = gs_in[0].color;
    for(int i=0; i<N; i+=2) {
        float a = radians(180.*(i+1)/N);
        float sa = sin(a)*(1.), ca = cos(a)*(1.);
        gl_Position = vec4(view_matrix * vec3(gs_in[0].position+sa*gs_in[0].right+ca*gs_in[0].forward, 1), -zorder/100., 1);
        EmitVertex();
        gl_Position = vec4(view_matrix * vec3(gs_in[0].position-sa*gs_in[0].right+ca*gs_in[0].forward, 1), -zorder/100., 1);
        EmitVertex();
    }
    EndPrimitive();
}
"""

CAR_GS = """{{HEAD}}
layout(points) in;
layout(triangle_strip, max_vertices = 12) out;
uniform vec3 front_color = vec3(0,0,0);
void main(){
    // Body
    gs_out.color = gs_in[0].color;
    for (int ca=-1; ca<=1; ca+=2)
        for (int sa=-1; sa<=1; sa+=2) {
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position+sa*gs_in[0].right+ca*gs_in[0].forward, 1), -zorder/100., 1);
            EmitVertex();
        }
    EndPrimitive();
    
    // Front
    gs_out.color = front_color;
    float arc = 0.75;
#define N 4
    for(int i=0; i<N; i++) {
        float a = radians(90.*i/(N-1));
        float sa = arc*sin(a), ca = arc-arc*cos(a);
        gl_Position = vec4(view_matrix * vec3(gs_in[0].position+(1-ca)*gs_in[0].right+(1+sa)*gs_in[0].forward, 1), -zorder/100., 1);
        EmitVertex();
        gl_Position = vec4(view_matrix * vec3(gs_in[0].position+(ca-1)*gs_in[0].right+(1+sa)*gs_in[0].forward, 1), -zorder/100., 1);
        EmitVertex();
    }
    EndPrimitive();
}
"""

BIKE_GS = """{{HEAD}}
layout(points) in;
layout(triangle_strip, max_vertices = 12) out;
uniform vec3 front_color = vec3(0,0,0);
void main(){
    // Body
    gs_out.color = gs_in[0].color;
    float w = 0.5;
    for (int ca=-1; ca<=1; ca+=2)
        for (int sa=-1; sa<=1; sa+=2) {
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position+w*sa*gs_in[0].right+ca*gs_in[0].forward, 1), -zorder/100., 1);
            EmitVertex();
        }
    EndPrimitive();
    
    // Wheels
    gs_out.color = front_color;
    float l = 0.2;
    for (int s=-1; s<=1; s+=2) {
        for (int ca=-1; ca<=1; ca+=2)
            for (int sa=-1; sa<=1; sa+=2) {
                gl_Position = vec4(view_matrix * vec3(gs_in[0].position+sa*gs_in[0].right+((1-l)*s+l*ca)*gs_in[0].forward, 1), -zorder/100.+1e-5, 1);
                EmitVertex();
            }
        EndPrimitive();
    }
}
"""