from .recording import Configuration, record, scenario
from .util import tqdm


def main():
    from argparse import ArgumentParser
    from .util import add_client_argument, client_from_args

    # Setup the standard ArgumentParser
    parser = ArgumentParser()
    add_client_argument(parser)
    Configuration.add_argument(parser)

    # Recording setting
    parser.add_argument('-s', '--max_steps', type=int, default=100000)
    parser.add_argument('output')
    args = parser.parse_args()

    # Create the config
    cfg = Configuration.from_args(args)

    # Start the client
    client, traffic_manager = client_from_args(args)

    # Start recording the scenario
    with scenario.scenario(client, cfg.scenario, traffic_manager) as world:
        with record(client, args.output):
            # TODO: Add sensors?
            try:
                for it in tqdm(range(args.max_steps)):
                    world.tick()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
