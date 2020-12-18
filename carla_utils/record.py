from .recording import Configuration, record, scenario
from .util import tqdm


def main():
    from argparse import ArgumentParser
    import carla

    parser = ArgumentParser()

    # Client settings
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tm_port', type=int, default=8000)
    parser.add_argument('--timeout', type=float, default=2.0)

    # Scenario settings
    parser.add_argument('--config', nargs='+', default=[])
    parser.add_argument('--config_override', nargs='+', default=[])

    # Recording setting
    parser.add_argument('-s', '--max_steps', type=int, default=100000)
    parser.add_argument('output')

    args = parser.parse_args()

    # Create the config
    cfg = Configuration.from_file(*args.config)
    for o in args.config_override:
        exec(o, {'config': cfg})

    # Start the client
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    traffic_manager = client.get_trafficmanager(args.tm_port)

    with scenario.scenario(client, cfg.scenario, traffic_manager) as world:
        with record(client, args.output):
            try:
                for it in tqdm(range(args.max_steps)):
                    world.tick()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
