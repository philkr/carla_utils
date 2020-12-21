import pathlib

from .util import tqdm
from .recording import Configuration, replay, sensors


def main():
    from argparse import ArgumentParser
    from .util import add_client_argument, client_from_args

    # Setup the standard ArgumentParser
    parser = ArgumentParser()
    add_client_argument(parser)
    Configuration.add_argument(parser)

    # Recording setting
    parser.add_argument('-s', '--max_steps', type=int, default=100000)
    parser.add_argument('recording', type=lambda x: str(pathlib.Path(x).resolve()))
    parser.add_argument('output_path')
    args = parser.parse_args()

    # Create the config
    cfg = Configuration.from_args(args)

    # Start the client
    client, traffic_manager = client_from_args(args)

    # Start recording the scenario
    with replay(client, args.recording) as world:
        with sensors(world, cfg.render, cfg.sensor, args.output_path):
            try:
                for _ in tqdm(range(min(args.max_steps, world.max_tick))):
                    world.tick()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
