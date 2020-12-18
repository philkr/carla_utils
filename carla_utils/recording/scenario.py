from contextlib import contextmanager
from .config import Configuration, Required, Settings
import random
from typing import List


__all__ = ['scenario', 'ScenarioSettings']


@Configuration.register('scenario')
class ScenarioSettings(Settings):
    class Agent(Settings):
        filter: str = 'vehicle.*'
        n: int = 0
        cars_only: bool = False
        role_name: str = 'autopilot'
        autopilot: bool = True

    # Agent settings
    seed: int = None
    agents: List[Agent] = [
        Agent(filter='vehicle.*', n=10, cars_only=True),
    ]

    # World settings
    map: str = 'town03'
    weather: str = None

    # Simulator
    synchronous_mode = True
    no_rendering_mode = True
    fixed_delta_seconds = 0.1


def weather_presets():
    import carla
    import re
    return [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]


@contextmanager
def scenario(client, config: ScenarioSettings, traffic_manager=None):
    import carla
    try:
        # Load the map
        world = client.load_world(config.map)

        # Simulator settings
        ws = world.get_settings()
        ws.synchronous_mode = config.synchronous_mode
        ws.no_rendering_mode = config.no_rendering_mode
        ws.fixed_delta_seconds = config.fixed_delta_seconds
        world.apply_settings(ws)
        if traffic_manager is not None:
            traffic_manager.set_synchronous_mode(config.synchronous_mode)
            traffic_manager.set_global_distance_to_leading_vehicle(1.0)

        # Init the random seed
        rnd = random.Random(config.seed)

        # World settings
        if config.weather is None or str(config.weather).lower() == 'none':
            world.set_weather(getattr(carla.WeatherParameters, rnd.choice(weather_presets())))
        else:
            assert hasattr(carla.WeatherParameters, config.weather), 'Invalid weather "{}"'.format(config.weather)
            world.set_weather(getattr(carla.WeatherParameters, config.weather))

        # Get the spawn points
        spawn_points = world.get_map().get_spawn_points()
        rnd.shuffle(spawn_points)

        # Shortcuts for later
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        # Create the blueprints and spawn vehicles batch commands
        blueprints = world.get_blueprint_library()
        batch = []
        for ac in config.agents:
            vehicle_blueprints = blueprints.filter(ac.filter)
            if ac.cars_only:
                vehicle_blueprints = [x for x in vehicle_blueprints if int(x.get_attribute('number_of_wheels')) == 4]

            for i in range(ac.n):
                blueprint = rnd.choice(vehicle_blueprints)
                for a in blueprint:
                    if a.is_modifiable:
                        blueprint.set_attribute(a.id, rnd.choice(a.recommended_values))
                blueprint.set_attribute('role_name', ac.role_name)
                # Add the blueprint to the spawn  points
                if len(batch) < len(spawn_points):
                    transform = spawn_points[len(batch)]
                    batch.append(SpawnActor(blueprint, transform).
                                 then(SetAutopilot(FutureActor, ac.autopilot, traffic_manager.get_port())))

        # Spawn
        actors = []
        for response in client.apply_batch_sync(batch):
            if response.error:
                print('Failed to spawn actor ', response.error, response)
            else:
                actors.append(response.actor_id)

        # # -------------
        # # Spawn Walkers
        # # -------------
        # # some settings
        # percentagePedestriansRunning = 0.0      # how many pedestrians will run
        # percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # # 1. take all the random locations to spawn
        # spawn_points = []
        # for i in range(args.number_of_walkers):
        #     spawn_point = carla.Transform()
        #     loc = world.get_random_location_from_navigation()
        #     if (loc != None):
        #         spawn_point.location = loc
        #         spawn_points.append(spawn_point)
        # # 2. we spawn the walker object
        # batch = []
        # walker_speed = []
        # for spawn_point in spawn_points:
        #     walker_bp = random.choice(blueprintsWalkers)
        #     # set as not invincible
        #     if walker_bp.has_attribute('is_invincible'):
        #         walker_bp.set_attribute('is_invincible', 'false')
        #     # set the max speed
        #     if walker_bp.has_attribute('speed'):
        #         if (random.random() > percentagePedestriansRunning):
        #             # walking
        #             walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
        #         else:
        #             # running
        #             walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        #     else:
        #         print("Walker has no speed")
        #         walker_speed.append(0.0)
        #     batch.append(SpawnActor(walker_bp, spawn_point))
        # results = client.apply_batch_sync(batch, True)
        # walker_speed2 = []
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         walkers_list.append({"id": results[i].actor_id})
        #         walker_speed2.append(walker_speed[i])
        # walker_speed = walker_speed2
        # # 3. we spawn the walker controller
        # batch = []
        # walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        # for i in range(len(walkers_list)):
        #     batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        # results = client.apply_batch_sync(batch, True)
        # for i in range(len(results)):
        #     if results[i].error:
        #         logging.error(results[i].error)
        #     else:
        #         walkers_list[i]["con"] = results[i].actor_id
        # # 4. we put altogether the walkers and controllers id to get the objects from their id
        # for i in range(len(walkers_list)):
        #     all_id.append(walkers_list[i]["con"])
        #     all_id.append(walkers_list[i]["id"])
        # all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if config.synchronous_mode:
            world.tick()
        else:
            world.wait_for_tick()

        # # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # # set how many pedestrians can cross the road
        # world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        # for i in range(0, len(all_id), 2):
        #     # start walker
        #     all_actors[i].start()
        #     # set walk to random point
        #     all_actors[i].go_to_location(world.get_random_location_from_navigation())
        #     # max speed
        #     all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

        # print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

        yield world

    finally:
        # # stop walker controllers (list is [controller, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
        #     all_actors[i].stop()

        client.apply_batch([carla.command.DestroyActor(x) for x in actors])

        ws = world.get_settings()
        ws.synchronous_mode = False
        world.apply_settings(ws)
