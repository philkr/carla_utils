
def add_argument(parser):
    # Client settings
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--tm_port', type=int, default=8000)
    parser.add_argument('--timeout', type=float, default=2.0)


def from_args(args):
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    traffic_manager = client.get_trafficmanager(args.tm_port)
    return client, traffic_manager
