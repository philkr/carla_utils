import time
import pathlib
import carla

from contextlib import contextmanager
from .recording.parse import parse
from .sensors.camera import Camera
from .common import tqdm, safe_sync_mode


@contextmanager
def sensor_pool(world, target_id, **kwargs):
    """
    Spawns and cleans up after sensors.
    """
    actor = world.get_actor(target_id)
    pool = list()

    try:
        for sensor_id, sensor in kwargs.items():
            sensor_actor = world.spawn_actor(sensor.blueprint, sensor.transform, actor)
            sensor_actor.listen(sensor.callback)

            pool.append(sensor_actor)

        yield
    finally:
        # Need to wait for the sensor callbacks to catch up.
        time.sleep(10)

        for sensor_actor in pool:
            sensor_actor.destroy()

        pool.clear()


def save_data(sensor_dict, save_dir):
    save_dir.mkdir()

    for sensor_name, sensor in sensor_dict.items():
        sensor_dir = save_dir / sensor_name
        sensor_dir.mkdir()

        total = 0

        for data in tqdm(sensor):
            sensor.save(data, sensor_dir, total)
            total += 1

        print('%d frames saved to %s.' % (total, sensor_dir))


def main(recording, save_dir, target_id, fps, host, port):
    assert save_dir.parent.exists(), '%s is not a valid directory.' % save_dir
    assert not save_dir.exists(), '%s already exists.' % save_dir

    world_map, frames = parse(recording)

    # Skip the last second. CARLA hangs if we run to the end...
    ticks = len(frames) - fps

    # Find a random vehicle if target_id is not specified.
    if target_id == 0:
        if len(frames[0].vehicles) > 0:
            target_id = frames[0].vehicles[0].id
        else:
            target_id = frames[0].actors[0].id

    client = carla.Client(host, port, worker_threads=4)
    client.set_replayer_time_factor(1.0)

    world = client.load_world(world_map.name)
    blueprints = world.get_blueprint_library()

    # TODO: Make this from a config file.
    sensor_dict = {
            'rgb': Camera(blueprints, 512, 512, 90, -5.5, 0, 2.5, 0, 0),
            'rgb_left': Camera(blueprints, 512, 512, 90, -5.5, 0, 2.5, 0, -45),
            }

    with safe_sync_mode(client, world, fps=fps):
        print(client.replay_file(recording, 0.0, ticks / fps, target_id))

        # Tick once to start the recording.
        world.tick()

        with sensor_pool(world, target_id, **sensor_dict):
            for t in tqdm(range(ticks-1)):
                world.tick()

    save_data(sensor_dict, save_dir)


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('recording', type=lambda x: str(pathlib.Path(x).resolve()))
    parser.add_argument('--save_dir', type=lambda x: pathlib.Path(x).resolve(), required=True)
    parser.add_argument('--target_id', type=int, default=0)

    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)

    main(**vars(parser.parse_args()))
