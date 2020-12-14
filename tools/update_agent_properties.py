
try:
    from tqdm import tqdm
except ImportError:
    def tqdm(x, *args, **kwargs):
        return x


def main():
    from argparse import ArgumentParser
    import carla

    parser = ArgumentParser()

    # Client settings
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)

    args = parser.parse_args()

    # Start the client
    client = carla.Client(args.host, args.port)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    for b in blueprint_library:
        actor = world.try_spawn_actor(b, carla.Transform(carla.Location(z=100)))
        if actor is not None and hasattr(actor, 'bounding_box'):
            print('_prop[{!r}] = AgentProperty(extent={!s})'.format(b.id, actor.bounding_box.extent))
        actor.destroy()
        del actor
    client.reload_world()



if __name__ == "__main__":
    main()
