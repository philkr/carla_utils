import pathlib

from .common import tqdm
from .recording.replay import SensorConfiguration, replay


def main(override_args, recording, sensor_config, save_dir, fps, host, port):
    sensor_config = SensorConfiguration.from_files(sensor_config, save_dir)
    sensor_config.override(override_args)

    with replay(recording, host, port, fps) as (world, ticks):
        sensor_config.hook(world)

        for t in tqdm(range(ticks)):
            frame_number = world.tick()
            sensor_config.wait(frame_number)


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('recording', type=lambda x: str(pathlib.Path(x).resolve()))
    parser.add_argument('--save_dir', type=lambda x: pathlib.Path(x).resolve(), required=True)
    parser.add_argument('--sensor_config', type=pathlib.Path)

    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)

    args, override_args = parser.parse_known_args()

    main(override_args, **vars(args))
