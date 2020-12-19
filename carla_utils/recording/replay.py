from contextlib import contextmanager
from .parse import parse


class MaxTickWorld:
    def __init__(self, world, max_tick):
        self.max_tick = max_tick
        self.world = world

    def tick(self, *args, **kwargs):
        if self.max_tick > 0:
            self.max_tick -= 1
            return self.world.tick(*args, **kwargs)
        return None

    def __getattr__(self, item):
        return getattr(self.world, item)

    def __setattr__(self, name, item):
        return setattr(self.world, name, item)


@contextmanager
def replay(client, recording: str, fps: int = 10):
    # Load the recording
    world_map, frames = parse(recording)

    # Skip the last second. CARLA hangs if we run to the end...
    n_ticks = len(frames) - fps

    # Replay at real time
    client.set_replayer_time_factor(1.0)
    world = client.load_world(world_map.name)

    try:
        # Sync mode for now
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / fps
        world.apply_settings(settings)

        # Load the replay
        client.replay_file(recording, 0.0, n_ticks / fps, 0)

        # Tick once to start the recording.
        world.tick()

        yield MaxTickWorld(world, n_ticks)
    finally:
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)
