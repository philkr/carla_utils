# Compares carla and carla_utils map

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('map')
    args = parser.parse_args()

    print(args)

    from carla_utils import map
    import carla

    carla_map = carla.Map()
    utils_map = map.Map()

if __name__ == "__main__":
    main()
