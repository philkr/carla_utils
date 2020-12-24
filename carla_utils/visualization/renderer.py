import moderngl
import numpy as np
from PIL import Image
from typing import List, Optional, Type
from .shaders import GENERIC_FS, GENERIC_VS, GS_HEAD

__all__ = ['Renderer', 'RenderFunction', 'bounding_view_matrix', 'const_view_matrix', 'follow_view_matrix']


class RenderFunctionRegistry:
    _all = {}

    @staticmethod
    def register(cls):
        RenderFunctionRegistry._all[cls.__name__] = cls
        return cls

    @staticmethod
    def all():
        return list(RenderFunctionRegistry._all.values())


class RenderFunction(RenderFunctionRegistry):
    # Class properties
    render_type = moderngl.POINTS
    vertex_shader = GENERIC_VS
    geometry_shader = None
    fragment_shader = GENERIC_FS
    uniforms = {}

    # Instance variables
    _position, _right, _forward, _color = None, None, None, None
    _bo = None
    _vao = None

    # Bounding box format (minx, miny), (maxx, maxy)
    bounding_box = None

    def __init__(self):
        self._bo = {}

    def _update_geometry(self, world_map, frame):
        raise NotImplementedError()

    def update(self, ctx, world_map=None, frame=None):
        # Create the geometry
        what = self._update_geometry(world_map, frame)
        if what is None:
            what = ['position', 'right', 'forward', 'color']

        if self._position is not None:
            # Update the bounding box
            self.bounding_box = self._position.min(axis=0), self._position.max(axis=0)

            # Create or update the buffer objects
            for n in ['position', 'right', 'forward', 'color']:
                a = getattr(self, '_'+n)
                if a is not None and n not in self._bo:
                    self._bo[n] = ctx.buffer(a.astype('f4'))
                elif a is not None and n in what:
                    # TODO: Consider self._bo[n].orphan or reallocate here
                    # self._bo[n].orphan(a.size*4)
                    self._bo[n].write(a.astype('f4'))
                    # self._bo[n] = ctx.buffer(a.astype('f4'))

    def render(self, ctx, **uniforms):
        if self._position is not None:
            # Create the shader programs and add them to the vbo
            if self._vao is None:
                prog = ctx.program(
                        vertex_shader=self.vertex_shader,
                        geometry_shader=self.geometry_shader.replace('{{HEAD}}', GS_HEAD),
                        fragment_shader=self.fragment_shader)
                for k, v in self.uniforms.items():
                    if not isinstance(v, np.ndarray):
                        v = np.array(v)
                    prog[k].write(v.astype('f4'))
                self._vao = ctx.vertex_array(prog, [(v, '3f' if k == 'color' else '2f', k) for k, v in self._bo.items()])

            # Update the uniforms
            for k, v in uniforms.items():
                self._vao.program[k].write(v)
            self._vao.render(self.render_type)


class Renderer:
    def __init__(self, w: int = 512, h: int = 512, msao: int = 8,
                 render_functions: Optional[List[Type[RenderFunction]]] = None,
                 modern_gl_context_args=dict(standalone=True, backend='egl')):
        self._ctx = moderngl.create_context(**modern_gl_context_args)
        self._ctx.enable_only(moderngl.DEPTH_TEST)
        self._fbo = self._ctx.framebuffer(color_attachments=[self._ctx.texture((w, h), 4)])
        self._fbo_msaa = self._ctx.framebuffer(
            color_attachments=[self._ctx.renderbuffer((w, h), 4, samples=msao)],
            depth_attachment=self._ctx.depth_renderbuffer((w, h), samples=msao)
        )
        if render_functions is None:
            render_functions = RenderFunctionRegistry.all()
        self._renderers = [cls() for cls in render_functions]

    def render(self, world_map, frame, view_matrix=None):
        # Update
        for f in self._renderers:
            f.update(self._ctx, world_map, frame)

        # Generate the view matrix
        if view_matrix is None:
            view_matrix = bounding_view_matrix()

        # Setup the fbo and view matrix
        self._fbo_msaa.use()
        self._fbo_msaa.clear(0.99, 0.99, 0.99)
        vm = view_matrix(world_map, frame, self._renderers)
        # Render
        for r in self._renderers:
            r.render(self._ctx, view_matrix=vm)
        # Get the image back
        self._ctx.copy_framebuffer(src=self._fbo_msaa, dst=self._fbo)
        data = self._fbo.read(components=3)
        image = Image.frombytes('RGB', self._fbo.size, data)
        return image

    def save_image(self, world_map, frame, output, view_matrix=None):
        import imageio
        import numpy as np
        writer = imageio.get_writer(output)
        image = self.render(world_map, frame, view_matrix=view_matrix)
        writer.append_data(np.array(image))
        writer.close()

    def save_video(self, world_map, frames, output, view_matrix=None, fps=30):
        import imageio
        import numpy as np
        from ..util import tqdm
        writer = imageio.get_writer(output, fps=fps, quality=9)
        for f in tqdm(frames):
            image = self.render(world_map, f, view_matrix=view_matrix)
            writer.append_data(np.array(image))
        writer.close()


def bounding_view_matrix(margin=20):
    """View matrix produced by a bounding box around all renderers"""
    def wrapped(world_map, frame, renderers):
        mn, mx = np.array([1e10, 1e10]), -np.array([1e10, 1e10])
        for r in renderers:
            if r.bounding_box is not None:
                mn = np.minimum(r.bounding_box[0], mn)
                mx = np.maximum(r.bounding_box[1], mx)
        if (mn >= mx).any():
            viewport = (0, 0), (100, 100)
        else:
            s = max((mx - mn) / 2) + margin
            viewport = (mx + mn) / 2, (s, s)

        return np.array([1.0 / viewport[1][0], 0,
                         0, 1.0 / viewport[1][1],
                         -1.0 * viewport[0][0] / viewport[1][0], -1.0 * viewport[0][1] / viewport[1][1]], dtype='f4')
    return wrapped


def follow_view_matrix(actor_id, w, h, rotate=False):
    """View matrix following an object with id actor_id, and crop a rectangle of size w x h"""
    w2, h2 = w/2., h/2.

    def wrapped(world_map, frame, renderers):
        a = frame.actor_by_id(actor_id)
        if a is None:
            return np.array([1.0 / w2, 0,
                             0, 1.0 / h2,
                             0, 0], dtype='f4')
        up, right = -np.array([1, 0]), -np.array([0, 1])
        if rotate:
            up, right = a.right[:2], -a.forward[:2]
        return np.array([up[0] / w2, right[0] / h2,
                         up[1] / w2, right[1] / h2,
                         -up.dot(a.location[:2]) / w2,
                         -right.dot(a.location[:2]) / h2], dtype='f4')
    return wrapped


def const_view_matrix(x, y, w, h):
    """Constant view matrix centered at (x, y) with a view size of (w, h)"""
    def viewmat(world_map, frame, renderers):
        return np.array([2.0 / w, 0,
                         0, 2.0 / h,
                         -2 * x / w, -2 * y / h], dtype='f4')
    return viewmat
