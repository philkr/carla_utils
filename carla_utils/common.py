from contextlib import contextmanager


try:
    from tqdm import tqdm
except ImportError:
    def tqdm(x, *args, **kwargs):
        return x


@contextmanager
def safe_sync_mode(client, world, fps):
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        settings.fixed_delta_seconds = 1.0 / fps
        world.apply_settings(settings)

        yield
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
