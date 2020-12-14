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
    for (int a=-1; a<=1; a+=2)
        for (int b=-1; b<=1; b+=2) {
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position+a*gs_in[0].right+b*gs_in[0].forward, 1), -zorder/100., 1);
            EmitVertex();
        }
    EndPrimitive();
}
"""
FILLED_RECT_GS = """{{HEAD}}
layout(points) in;
layout(triangle_strip, max_vertices = 8) out;
uniform vec3 fill_color = vec3(0,0,0);
uniform float border_size = 0;
void main(){
    gs_out.color = fill_color;
    for (int s=0; s<=1; s++) {
        float e = border_size * s;
        vec2 r = gs_in[0].right + e * normalize(gs_in[0].right);
        vec2 f = gs_in[0].forward + e * normalize(gs_in[0].forward);
        for (int a=-1; a<=1; a+=2)
            for (int b=-1; b<=1; b+=2) {
                gl_Position = vec4(view_matrix * vec3(gs_in[0].position+a*r+b*f, 1), -zorder/100. + 1e-4*s, 1);
                EmitVertex();
            }
        EndPrimitive();
        gs_out.color = gs_in[0].color;
    }
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
layout(triangle_strip, max_vertices = 8) out;
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
    float front_portion = 0.25;
    for (int k=0; k<=1; k++)
        for (int sa=-1; sa<=1; sa+=2) {
            float ca = (1-2*front_portion*k);
            gl_Position = vec4(view_matrix * vec3(gs_in[0].position+sa*gs_in[0].right+ca*gs_in[0].forward, 1), -zorder/100.-1e-4, 1);
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
