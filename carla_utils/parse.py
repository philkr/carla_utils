from .recording import parse

if __name__ == "__main__":
    # Testing the parsing function
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('recording')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    world_map, frames = parse(args.recording)
    print('Map              {!r:>10}'.format(world_map.name))
    print('# frames         {!r:>10}'.format(len(frames)))
    for n in ['cars', 'bikes', 'walkers', 'traffic_lights', 'other_actors', 'invalid_actors']:
        print('# {:15}{!r:>10}'.format(n, len(getattr(frames[0], n))))
    if args.verbose:
        for n in ['cars', 'bikes', 'walkers', 'traffic_lights', 'other_actors', 'invalid_actors']:
            actor_ids = {x.id for f in frames for x in getattr(f, n)}
            if actor_ids:
                print('  {:15}{}'.format(n, ','.join([str(id) for id in sorted(list(actor_ids))])))
                actor_attributes = set.union(*[set(x.attributes) for f in frames for x in getattr(f, n)])
                if actor_attributes:
                    print('    attributes:', ', '.join(actor_attributes))
