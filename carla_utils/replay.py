import pathlib

from .common import tqdm
from .recording.replay import SensorConfiguration, replay


def main(recording, target_id, fps, host, port, sensor_config, save_dir):
    sensor_config = SensorConfiguration.from_files(sensor_config, save_dir)

    with replay(recording, host, port, fps) as (world, ticks):
        sensor_config.hook(world)

        for t in tqdm(range(ticks-1)):
            frame_number = world.tick()

        sensor_config.finalize(frame_number)


if __name__ == "__main__":
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('recording', type=lambda x: str(pathlib.Path(x).resolve()))
    parser.add_argument('--save_dir', type=lambda x: pathlib.Path(x).resolve(), required=True)
    parser.add_argument('--target_id', type=int, default=0)
    parser.add_argument('--sensor_config', type=pathlib.Path, nargs='+')

    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)

    main(**vars(parser.parse_args()))
