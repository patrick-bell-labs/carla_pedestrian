
import carla
import random
import time
import numpy as np
import sys
import os
import pdb
from PIL import Image
from pathlib import Path
import argparse

FPS=20
bootstrap_frame=20

class Sensors:
    def __init__(self, cameras, base_map_only):

        self.frame = 0
        self.base_map_only = base_map_only
        class CameraHandler:
            def __init__(self, camera, folder, camera_id, base_map_only):
                self.camera_id = camera_id
                self.folder = folder
                self.frame = 0
                self.camera = camera
                self.base_frame_id = None
                self.pointcloud_folder = 'point_cloud'
                self.base_map_only = base_map_only
                #assert sensor_type in ['rgb', 'depth'], 'Sensor type must be either rgb or depth'

            def __call__(self, image):
                if not self.base_map_only and self.frame < bootstrap_frame:
                    return
                #image = image.convert(carla.ColorConverter.Raw)
                array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
                array = np.reshape(array, (image.height, image.width, 4))
                # filename = f"{self.folder}/camera_{self.camera_id}_{self.frame:06d}.npy"
                # np.save(filename, array)

                if self.base_frame_id is None:
                    self.base_frame_id = image.frame

                frame_id = image.frame - self.base_frame_id
                #print(f'capturing base_map {self.base_map_only} frames for {frame_id}')
                array = array[:, :, :3]
                #array = array[:, :, 0]
                array = array[:, :, ::-1]  # BGR to RGB
                image_filename = str(self.folder/f"camera_{self.camera_id}_{frame_id:06d}.png")
                im = Image.fromarray(array)
                im.save(image_filename)

                """
                # Save the camera position
                # camera_transform = self.camera.get_transform()
                camera_transform = image.transform
                transform_matrix = np.array(camera_transform.get_matrix())
                #print(f'transform_matrix {transform_matrix}')
                #print(f'camera get_transform {self.camera.get_transform()}')
                transform_filename = str(self.folder/ f"transform_matrix_{self.camera_id}_{frame_id:06d}.npy")
                np.save(transform_filename, transform_matrix)
                """
            def tick(self):
                self.frame += 1

        self.camera_handlers = []
        for camera, folder, camera_id in cameras:
            #print(f'Saving image from camera {camera_id} to {folder}')
            ch = CameraHandler(camera, folder, camera_id, self.base_map_only)
            camera.listen(ch)
            self.camera_handlers.append(ch)
            #print(f'camera: {camera_id} transform: {camera.get_transform()}')
            # if "rgb" in folder:
            #     camera.listen(lambda image: self.save_image(image, folder, camera_id))
            # else:
            #     camera.listen(lambda image: self.save_depth(image, folder, camera_id))

    def tick(self):
        self.frame += 1
        for ch in self.camera_handlers:
            ch.tick()
    
def save_image(image, folder, frame, camera_id, image_type):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))

    filename = f"{folder}/{image_type}_{camera_id}_{frame:06d}.npy"
    np.save(filename, array)

    image_filename = f"{folder}/{image_type}_{camera_id}_{frame:06d}.png"
    array = array[:, :, :3]
    array = array[:, :, ::-1]  # BGR to RGB
    im = Image.fromarray(array)
    im.save(image_filename)

def get_intrinsic_matrix(sensor):
    """
    Calculate the intrinsic matrix for the given sensor.
    
    Parameters:
    sensor (carla.Sensor): The RGB camera sensor.
    
    Returns:
    np.ndarray: The intrinsic matrix.
    """
    # Get the camera attributes
    image_width = sensor.get_attribute('image_size_x').as_int()
    image_height = sensor.get_attribute('image_size_y').as_int()
    fov = float(sensor.get_attribute('fov'))

    # Calculate the focal length in pixels
    focal_length = image_width / (2.0 * np.tan(np.deg2rad(fov) / 2.0))

    # Construct the intrinsic matrix
    intrinsic_matrix = np.array([
        [focal_length, 0, image_width / 2.0],
        [0, focal_length, image_height / 2.0],
        [0, 0, 1]
    ])

    return intrinsic_matrix

def get_safe_spawn_points(world, num_points, min_distance=2.0):
    spawn_points = []
    attempts = 0
    max_attempts = num_points * 10

    while len(spawn_points) < num_points and attempts < max_attempts:
        location = world.get_random_location_from_navigation()
        print(f'location from nav {location}')
        if location is None:
            continue

        too_close = False
        for point in spawn_points:
            if location.distance(point.location) < min_distance:
                too_close = True
                break

        if not too_close:
            spawn_points.append(carla.Transform(location))

        attempts += 1

    return spawn_points

def get_collision_free_spawn_points(world, num_points, cam_location, min_distance=2.0):
    navmesh = carla.Location()
    spawn_points = []
    existing_locations = []

    map = world.get_map()
    debug = world.debug

    attempts = 0
    max_attempts = num_points * 10
    # allowed maximum distance from camera to spawn point
    MAX_CAM_DISTANCE = 100.0 
    while len(spawn_points) < num_points and attempts < max_attempts:
        loc = world.get_random_location_from_navigation()
        attempts += 1
        if loc is None:
            continue

        # Slight Z offset to avoid underground spawn
        loc.z += 0.5

        # Check distance to existing points
        if any(loc.distance(existing) < min_distance for existing in existing_locations):
            #print('this point is too close to another')
            continue
        
        # Check distance to camera position
        if loc.distance(cam_location) > MAX_CAM_DISTANCE:
            #print('this point is too far from the camera')
            continue

        # Raycast to check if there is ground below (prevent floating spawns)
        ray_start = loc + carla.Location(z=2.0)
        ray_end = loc - carla.Location(z=2.0)
        hit_results = world.cast_ray(ray_start, ray_end)

        if hit_results:
            nearest_hit = hit_results[0]
            if abs(nearest_hit.location.z - loc.z) < 1.0:
        		# Valid spawn point
				# Draw point for debug
            	#debug.draw_point(loc, 0.1, carla.Color(0, 255, 0), 5.0)
            	spawn_points.append(carla.Transform(loc))
            	existing_locations.append(loc)
            #else:
                # Draw failed attempt
                #debug.draw_point(loc, 0.1, carla.Color(255, 0, 0), 2.0)

    return spawn_points

def sim_dynamic():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    #world = client.load_world('Town10_Opt')
    world = client.get_world()
    world_settings = world.get_settings()
    world_settings.synchronous_mode = True
    world_settings.fixed_delta_seconds = 1/FPS
    world.apply_settings(world_settings)

    blueprint_library = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    print(f'Found {len(spawn_points)} spawn points')

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_random_device_seed(24)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(True)

    vehicle_list = []
    walker_list = []

    # Create output folders
    output_folder = Path("camera_capture_output")
    output_folder.mkdir(exist_ok=True)
    rgb_folder = output_folder / "rgb"
    depth_folder = output_folder / "depth"
    segment_folder = output_folder / "segmentation"

    rgb_folder.mkdir(exist_ok=True)
    depth_folder.mkdir(exist_ok=True)
    segment_folder.mkdir(exist_ok=True)

    try:

        # Spawn special exploring walker
        explorer_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
        explorer_spawn_point = random.choice(spawn_points)
        spawn_points.remove(explorer_spawn_point)
        #explorer = world.spawn_actor(explorer_bp, explorer_spawn_point)

        print('Creating explorer vehicle')
        explorer_vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
        if explorer_vehicle_bp.has_attribute('color'):
            color = random.choice(explorer_vehicle_bp.get_attribute('color').recommended_values)
            explorer_vehicle_bp.set_attribute('color', color)

        if len(spawn_points) == 0:
            raise Exception("No more spawn points left")
        
        spawn_point = random.choice(spawn_points)
        spawn_points.remove(spawn_point)
        explorer_vehicle = world.try_spawn_actor(explorer_vehicle_bp, spawn_point)
        
        if explorer_vehicle is None:
            raise Exception("Failed to spawn special exploring vehicle")
        explorer_vehicle.set_autopilot(True, traffic_manager.get_port())
            #explorer_vehicle_list.append(vehicle)
        
        #################################################################################
        # Create the four cameras for the special exploring vehicle
        print('Creating cameras for special exploring vehicle')
        camera_bps = [
            (blueprint_library.find('sensor.camera.rgb'), rgb_folder),
            (blueprint_library.find('sensor.camera.depth'), depth_folder),
            (blueprint_library.find('sensor.camera.instance_segmentation'), segment_folder)
        ]

        focal_length = 1000

        for camera_bp, folder in camera_bps:
            camera_bp.set_attribute('image_size_x', '2048')
            camera_bp.set_attribute('image_size_y', '1080')
            camera_bp.set_attribute('fov', '100')
            camera_bp.set_attribute('lens_k', '0')
            camera_bp.set_attribute('lens_kcube', '0')
            camera_bp.set_attribute('lens_circle_falloff', '0')

            if camera_bp.has_attribute('bloom_intensity'):
                camera_bp.set_attribute('bloom_intensity', '0')

            if camera_bp.has_attribute('motion_blur_intensity'):
                camera_bp.set_attribute('motion_blur_intensity', '0')

            if camera_bp.has_attribute('focal_distance'):
                camera_bp.set_attribute('focal_distance', str(focal_length))

            # Save the intrinsic matrix for the camera
            intrinsic_matrix = get_intrinsic_matrix(camera_bp)
            intrinsic_filename = folder / 'intrinsic_matrix.npy'
            np.save(str(intrinsic_filename), intrinsic_matrix)
            print(f'Saved intrinsic matrix to {intrinsic_filename}')

        cameras = []

        for i, direction in enumerate([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]):
            
            for camera_bp, folder in camera_bps:
                # camera_transform = carla.Transform(carla.Location(x=0.5*direction[0], y=0.5*direction[1], z=2.5), 
                #                                    carla.Rotation(pitch=0, yaw=90*i, roll=0))
                
                camera_transform = carla.Transform(carla.Location(x=0, y=0, z=2.5), 
                                                   carla.Rotation(pitch=0, yaw=90*i, roll=0))

                
                m = camera_transform.get_matrix()
                # transform_matrix = np.array(m)
                # transform_filename = os.path.join(folder, f'transform_matrix_{i}.npy')
                # np.save(transform_filename, transform_matrix)
                # print(f'Saved transform matrix for camera {i} to {transform_filename}')

                camera = world.spawn_actor(camera_bp, camera_transform, attach_to=explorer_vehicle)
                cameras.append((camera, folder, i))
        ################################################################################# 

        
        # # Set up explorer's AI controller
        # explorer_controller_bp = blueprint_library.find('controller.ai.walker')
        # explorer_controller = world.spawn_actor(explorer_controller_bp, carla.Transform(), explorer)
        # explorer_controller.start()
        # explorer_target_location = world.get_random_location_from_navigation()
        # explorer_controller.go_to_location(explorer_target_location)

        sensors = Sensors(cameras)

        #print("Spawned special exploring walker with 4 cameras")
        # Spawn vehicles (same as before)
        for i in range(100):
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
            if vehicle_bp.has_attribute('color'):
                color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
                vehicle_bp.set_attribute('color', color)

            if len(spawn_points) == 0:
                raise Exception("No more spawn points left")
            
            spawn_point = random.choice(spawn_points)
            spawn_points.remove(spawn_point)
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
            
            if vehicle is not None:
                vehicle.set_autopilot(True, traffic_manager.get_port())
                vehicle_list.append(vehicle)

        print(f"Spawned {len(vehicle_list)} vehicles")

        # # Spawn regular walkers (same as before)
        # for i in range(50):
        #     walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
        #     if len(spawn_points) == 0:
        #         raise Exception("No more spawn points left")
        #     sp_tmp = random.choice(spawn_points)
        #     spawn_points.remove(sp_tmp)
        #     spawn_point = carla.Transform(sp_tmp.location + carla.Location(z=0.5))
        #     walker = world.try_spawn_actor(walker_bp, spawn_point)
        #     
        #     if walker is not None:
        #         walker_controller_bp = blueprint_library.find('controller.ai.walker')
        #         walker_controller = world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
        #         walker_controller.start()
        #         walker_list.append((walker, walker_controller))

        # print(f"Spawned {len(walker_list)} regular walkers")

        frame = 0
        while True:
            print(f'Frame {frame}')
            sensors.tick()
            world.tick()

            # # Update explorer's destination if it's close to the current target
            # if explorer.get_location().distance(explorer_target_location) < 1.0:
            #     explorer_target_location = world.get_random_location_from_navigation()
            #     explorer_controller.go_to_location(explorer_target_location)
            #     print(f'Explorer reached target, setting new target: {explorer_target_location}')

            # # Save camera outputs
            # for camera, folder, camera_id in cameras:
            #     print(f'Saving image from camera {camera_id} to {folder}')
            #     camera.listen(lambda image, f=folder, cid=camera_id: 
            #         save_image(image, f, frame, cid, "rgb" if "rgb" in f else "depth"))

            frame += 1
            time.sleep(0.1)

    finally:
        print("Cleaning up spawned actors...")
        for camera, _, _ in cameras:
            camera.destroy()
        #explorer_controller.stop()
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        carla.command.DestroyActor(explorer_vehicle)
        # for walker, controller in walker_list:
        #     controller.stop()
        #client.apply_batch([carla.command.DestroyActor(walker) for walker, _ in walker_list])
        #client.apply_batch([carla.command.DestroyActor(controller) for _, controller in walker_list])
        #explorer.destroy()
        #explorer_controller.destroy()
        print("Cleanup complete.")

def debug_walkers(world, destroy = False):
    walker_list = world.get_actors().filter('walker.*')
    print(f"Number of walkers actually spawned: {len(walker_list)}")
    for walker in walker_list:
        location = walker.get_location()
        #print(f"Walker ID: {walker.id} at location: {location}")
        if destroy and location.x == 0.0:
            print("destroying this walker")
            walker.destroy()

def get_random_speed(speed_class='slow'):
    #walking speed = {0.5-1.0, 1.0-1.5, 1.5-2.0}
    if (speed_class == 'slow'):
        return random.uniform(0.5, 1.0)
    elif (speed_class == 'normal'):
        return random.uniform(1.0, 1.5)
    elif (speed_class == 'fast'):
        return random.uniform(1.5, 2.0)
    else:
        print("ERROR: speed class string not supported")
        return None

# weather: 'morning', 'afternoon', 'late_afternoon', 'evening'
def get_world_weather(weather='morning'):
    # Morning preset (default)
    morning_weather = carla.WeatherParameters(
        cloudiness=20.0,
        precipitation=0.0,
        sun_altitude_angle=25.0,
        sun_azimuth_angle=90.0,
        fog_density=10.0,
        wetness=0.0
    )

    # Afternoon preset
    afternoon_weather = carla.WeatherParameters(
        cloudiness=10.0,
        precipitation=0.0,
        sun_altitude_angle=75.0,
        sun_azimuth_angle=180.0,
        fog_density=0.0,
        wetness=0.0
    )


    # Late afternoon preset (4-6 PM)
    late_afternoon_weather = carla.WeatherParameters(
        cloudiness=15.0,           # Light clouds for interesting shadows
        precipitation=0.0,
        sun_altitude_angle=30.0,   # Medium-low angle for golden light
        sun_azimuth_angle=240.0,   # Southwest position
        fog_density=5.0,           # Slight atmospheric haze
        wetness=0.0,
        wind_intensity=20.0,       # Gentle breeze
        fog_distance=100.0,        # Atmospheric depth
        scattering_intensity=1.2   # Enhanced light scattering
    )

    # Evening preset
    evening_weather = carla.WeatherParameters(
        cloudiness=30.0,
        precipitation=0.0,
        sun_altitude_angle=5.0,
        sun_azimuth_angle=270.0,
        fog_density=15.0,
        wetness=0.0
    )

    if (weather == 'morning'):
        return morning_weather
    elif (weather == 'afternoon'):
        return afternoon_weather
    elif (weather == 'late_afternoon'):
        return late_afternoon_weather
    elif (weather == 'evening'):
        return evening_weather
    else:
        print("ERROR: weather string not supported")
        return None

def safe_cleanup_with_sensors(world):
    """
    Cleanup actors in the correct order to avoid stream errors
    """
    all_actors = world.get_actors()

    # 1. Stop and destroy sensors FIRST
    sensors = all_actors.filter('sensor.*')
    for sensor in sensors:
        try:
            sensor.stop()  # Stop streaming
            sensor.destroy()
        except RuntimeError:
            pass  # Sensor already destroyed

    # 2. Destroy controllers
    controllers = all_actors.filter('controller.*')
    for controller in controllers:
        try:
            controller.destroy()
        except RuntimeError:
            pass

    # 3. Destroy vehicles and walkers
    vehicles = all_actors.filter('vehicle.*')
    walkers = all_actors.filter('walker.*')

    for actor in list(vehicles) + list(walkers):
        try:
            actor.destroy()
        except RuntimeError:
            pass

    # 4. Process destruction in synchronous mode
    if world.get_settings().synchronous_mode:
        world.tick()

    print("Safe cleanup completed")

def cleanup_all_simulation_actors(world):
    """
    Destroys all spawned actors except traffic lights, signs, etc.
    """
    all_actors = world.get_actors()
    
    # Filter out static/essential actors
    actors_to_destroy = list(all_actors.filter('vehicle.*')) + \
                        list(all_actors.filter('walker.*')) + \
                        list(all_actors.filter('controller.*')) + \
                        list(all_actors.filter('sensor.*'))
    
    destroyed_count = 0
    for actor in actors_to_destroy:
        actor.destroy()
        destroyed_count += 1
    
    print(f"Destroyed {destroyed_count} actors")
    return destroyed_count


def sim_static(_points, _fov, _camera_height, _speed, _weather, FPS=30, NUM_FRAMES=200):

    """ 
    In this scenario, we spawn a stationary pedestrian and let the cars drive around it
    """
    base_map_only=False 
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    #print(client.get_available_maps())
    #world = client.load_world('/Game/Carla/Maps/Town10HD_Opt')
    world = client.get_world()
    #world.generate_navigation()
    world_settings = world.get_settings()
    world_settings.synchronous_mode = True
    world_settings.fixed_delta_seconds = 1/FPS
    world.apply_settings(world_settings)

    world.set_weather(get_world_weather(_weather))

    blueprint_library = world.get_blueprint_library()
    # Print available blueprint categories
    #for bp in blueprint_library:
    #    print(f'blueprint library: {bp.id}')
    
    spawn_points = world.get_map().get_spawn_points()
    print(f'Found {len(spawn_points)} spawn points')

    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_random_device_seed(24)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(True)

    vehicle_list = []
    walker_list = []

    # Create output folders
    output_folder = Path("camera_capture_output")
    if base_map_only:
        output_folder = Path("base_map")

    output_folder.mkdir(exist_ok=True)
    rgb_folder = output_folder / "rgb"
    #depth_folder = output_folder / "depth"
    #segment_folder = output_folder / "segmentation"

    rgb_folder.mkdir(exist_ok=True)
    #depth_folder.mkdir(exist_ok=True)
    #segment_folder.mkdir(exist_ok=True)

    try:
        actor_list = world.get_actors()
        # Print unique actor types
        actor_types = set(actor.type_id for actor in actor_list)
        print("Available actor types:", actor_types)
        # Try different patterns
        patterns = [
            'static.*',
            'static.prop.*',
            'static.vegetation.*',
            'mesh.*',
            'foliage.*'
        ]
        for pattern in patterns:
            actors = world.get_actors().filter(pattern)
            print(f"Pattern {pattern}: {len(actors)} actors found")
            for actor in actors:
                print(f"Found: {actor.type_id}")
        
        for actor in actor_list:
            #print(f'{actor.type_id}')
            #if actor.type_id.startswith('static.prop'):  # or specific prop names
            if 'static' in actor.type_id:
                print(actor)
                actor.destroy()
        
        #################################################################################
        # Find the waypoints of the traffic junction
        map = world.get_map()
        waypoints = map.generate_waypoints(distance=1.0)
        # Identify junction locations
        junction_locations = []
        junction_candidates = {}
        unique_locations = []
        distance_threshold = 2.0
        for waypoint in waypoints:
            if waypoint.is_junction:
                #print(f'unique_locations len {len(unique_locations)}')
                junction_locations.append(waypoint.transform.location)
                location_key = waypoint.transform.location
                if not any(location_key.distance(unique_loc) < distance_threshold for unique_loc in unique_locations):
                    junction_candidates[location_key] = []
                    junction_candidates[location_key].append(waypoint)
                    unique_locations.append(location_key)
                else:
                    min_dist = 10000
                    loc_key = None
                    for unique_loc in unique_locations:
                        if location_key.distance(unique_loc) < min_dist:
                            min_dist = location_key.distance(unique_loc)
                            loc_key = unique_loc
                    junction_candidates[loc_key].append(waypoint)
      	# Find a junction with at least four unique entry directions
        selected_locations = []
        for loc, wps in junction_candidates.items():
            directions = set()
            for wp in wps:
                yaw = round(wp.transform.rotation.yaw / 90) % 4
                directions.add(yaw)
            #print(f'this junction at {loc} has {len(directions)} ways')
            if len(directions) >= 4:
                print(f"Likely four-way intersection at: {loc}")
                selected_locations.append(loc)
                #break
                
        #camera_location = unique_junction_locations[17]
        print(selected_locations)
        camera_location = selected_locations[1]
        print(camera_location)
        #sys.exit()
        #################################################################################
        # Create the four cameras for the special exploring vehicle
        print('Creating cameras for special exploring vehicle')
        camera_bps = [
            (blueprint_library.find('sensor.camera.rgb'), rgb_folder),
            #(blueprint_library.find('sensor.camera.depth'), depth_folder),
            #(blueprint_library.find('sensor.camera.instance_segmentation'), segment_folder)
        ]

        focal_length = 2000

        for camera_bp, folder in camera_bps:
            ## 720p cameras
            #camera_bp.set_attribute('image_size_x', '1280')
            #camera_bp.set_attribute('image_size_y', '720')
            ## Full HD cameras
            camera_bp.set_attribute('image_size_x', '2048')
            camera_bp.set_attribute('image_size_y', '1080')
            ## 4K cameras
            #camera_bp.set_attribute('image_size_x', '4096')
            #camera_bp.set_attribute('image_size_y', '2160')
            camera_bp.set_attribute('fov', _fov)
            camera_bp.set_attribute('lens_k', '0')
            camera_bp.set_attribute('lens_kcube', '0')
            camera_bp.set_attribute('lens_circle_falloff', '0')

            if camera_bp.has_attribute('bloom_intensity'):
                camera_bp.set_attribute('bloom_intensity', '0')

            if camera_bp.has_attribute('motion_blur_intensity'):
                camera_bp.set_attribute('motion_blur_intensity', '0')

            if camera_bp.has_attribute('focal_distance'):
                camera_bp.set_attribute('focal_distance', str(focal_length))

            # Save the intrinsic matrix for the camera
            intrinsic_matrix = get_intrinsic_matrix(camera_bp)
            intrinsic_filename = folder / 'intrinsic_matrix.npy'
            np.save(str(intrinsic_filename), intrinsic_matrix)
            print(f'Saved intrinsic matrix to {intrinsic_filename}')
            print(f'Camera bp fov {camera_bp.get_attribute("fov")}')

        cameras = []
        camera_height = _camera_height
        
        camera_location_xyz = np.array([camera_location.x, camera_location.y, camera_height])
        np.savetxt(output_folder / "camera_location_xyz_unreal.txt", camera_location_xyz)
        
        cam_location_carla = carla.Transform(carla.Location(x=camera_location.x, y=camera_location.y, z=0.0))
        # camera tuple (x_offset, y_offset, yaw_degree)
        # refer to the powerpoint for more info
        camera_profiles = [(35, -15, 0), (35,15, 0),
                            (20, -15, 0), (20, 15, 0),
                            (-17,-35, 270), (-17,-15, 270), (-17, 15, 90), (-17, 35, 90),
                            (-20, -15, 180), (-20, 15, 180),
                            (-35, -15, 180), (-35, 15, 180)]
        for i, cp in enumerate(camera_profiles):
            cam_pos_offset_x = cp[0]
            cam_pos_offset_y = cp[1]
            yaw_degree = cp[2]
            camera_transform = carla.Transform(
                                    carla.Location( x=camera_location.x+cam_pos_offset_x, 
                                                    y=camera_location.y+cam_pos_offset_y, 
                                                    z=camera_height), 
                                    carla.Rotation(pitch=0, yaw=yaw_degree, roll=0))
            m = camera_transform.get_matrix()
            camera = world.spawn_actor(camera_bp, camera_transform)
            cameras.append((camera, folder, i+1)) # start from camera index 1

        """
        index_offset = 0
        degree_step = 90
        cam_pos_offset_x = -17
        cam_pos_offset_y = -15
        #for i, direction in enumerate([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]):
        for i in range(4): 
            for camera_bp, folder in camera_bps:
                print(f'index {i}')
                # camera_transform = carla.Transform(carla.Location(x=0.5*direction[0], y=0.5*direction[1], z=2.5), 
                #                                    carla.Rotation(pitch=0, yaw=90*i, roll=0))
                
                # camera_transform = carla.Transform(carla.Location(x=0, y=0, z=2.5), 
                                                   # carla.Rotation(pitch=0, yaw=90*i, roll=0))
                camera_transform = carla.Transform(carla.Location(x=camera_location.x+cam_pos_offset_x, 
                                                                    y=camera_location.y+cam_pos_offset_y, 
                                                                    z=camera_height), 
                                                   carla.Rotation(pitch=0, yaw=degree_step*i, roll=0))

                
                m = camera_transform.get_matrix()
                # transform_matrix = np.array(m)
                # transform_filename = os.path.join(folder, f'transform_matrix_{i}.npy')
                # np.save(transform_filename, transform_matrix)
                # print(f'Saved transform matrix for camera {i} to {transform_filename}')

                #camera = world.spawn_actor(camera_bp, camera_transform, attach_to=explorer_vehicle)
                camera = world.spawn_actor(camera_bp, camera_transform)

                cameras.append((camera, folder, i+index_offset))
        
        """
        print(f'total number of cameras {len(cameras)}')
        sensors = Sensors(cameras, base_map_only)
        ################################################################################# 
        
        if not base_map_only:
            # Spawn vehicles (same as before)
            max_vehicle = 10 # 140 for dense scene
            for car_id, spawn_point in enumerate(spawn_points):
                if len(vehicle_list) >= max_vehicle:
                    break
                vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
                #vehicle_bp = blueprint_library.filter('vehicle.dodge.charger_police_2020')
                if vehicle_bp.has_attribute('color'):
                    color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
                    vehicle_bp.set_attribute('color', color)

                if len(spawn_points) == 0:
                    raise Exception("No more spawn points left")

                # spawn_point = random.choice(spawn_points)
                # spawn_points.remove(spawn_point)
                vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)

                if vehicle is not None:
                    vehicle.set_autopilot(True, traffic_manager.get_port())
                    vehicle_list.append(vehicle)

            print(f"Spawned {len(vehicle_list)} vehicles")
            # Spawn pedestrains
            # Set to 200 for ~110 pedestrians 
            # Set to 400 for ~220 pedestrians 
            # Set to 600 for ~340 pedestrains
            MAX_PEDESTRIAN_SPAWN_POINTS = _points
            pedestrian_bp = blueprint_library.filter('walker.pedestrian.*')
            walker_controller_bp = blueprint_library.find('controller.ai.walker')
            print(f'Max pedestrian spawn points {MAX_PEDESTRIAN_SPAWN_POINTS}')
            #spawn_points = get_safe_spawn_points(world, NUMBER_OF_PEDESTRIANS, 1.0)
            spawn_points = get_collision_free_spawn_points(world, MAX_PEDESTRIAN_SPAWN_POINTS, cam_location_carla.location, 2.0)
            print(f'pedestrain spawn points {len(spawn_points)}')
            #for point in spawn_points:
            #    print(f'spawn point {point}')
            
			# List of (walker, controller) actor IDs to keep track
            batch = []
            walker_ids = []
            walker_controller_ids = []

            for spawn_point in spawn_points:
                walker_bp = random.choice(pedestrian_bp)
                walker_bp.set_attribute('is_invincible', 'true')
                batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

            responses = client.apply_batch_sync(batch, True)
            #responses = client.apply_batch(batch)

            for response in responses:
                if response.error:
                    print(f"Error: {response.error}")
                else:
                    walker_ids.append(response.actor_id)

            # Spawn controllers for each walker
            batch = []
            for walker_id in walker_ids:
                walker_actor = world.get_actor(walker_id)
                batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walker_actor))

            responses = client.apply_batch_sync(batch, True)
            for response in responses:
                if response.error:
                    print(f"Controller Error: {response.error}")
                else:
                    walker_controller_ids.append(response.actor_id)

			# Let everything settle
            time.sleep(2)

			# Start walker controllers
            all_actors = world.get_actors(walker_ids + walker_controller_ids)
            walkers = all_actors.filter('walker.pedestrian.*')
            controllers = all_actors.filter('controller.ai.walker')
            
            num_valid_controllers = 0
            for controller in controllers:
                found_valid_location = False
                while not found_valid_location:
                    destination = world.get_random_location_from_navigation()
                    if destination:
                        controller.start()
                        controller.go_to_location(destination)
                        # walking speed = {slow: 0.5-1.0, normal: 1.0-1.5, fast: 1.5-2.0}
                        controller.set_max_speed( get_random_speed( _speed ) )  # default to slow
                        found_valid_location = True
                        num_valid_controllers += 1

            print(f"Spawned {len(walkers)} walkers {len(controllers)} pedestrian controllers with {num_valid_controllers} being valid.")
        else:
            print('Base map only. No vehicles or walkers spawned')

        frame = 0
        # max number of frames + boost strap frames (cars being spawned) that skip sensor captures
        max_frame = NUM_FRAMES
        while True:
            print(f'Frame {frame}')
            world.tick()
            sensors.tick() 
            #debug_walkers(world)    
            #for vehicle in vehicle_list:
            #    bbox = vehicle.bounding_box
            #    print(bbox.location)
            #    print(bbox.extent)
            #    print(bbox.rotation)
            
            # # Update explorer's destination if it's close to the current target
            # if explorer.get_location().distance(explorer_target_location) < 1.0:
            #     explorer_target_location = world.get_random_location_from_navigation()
            #     explorer_controller.go_to_location(explorer_target_location)
            #     print(f'Explorer reached target, setting new target: {explorer_target_location}')

            # # Save camera outputs
            # for camera, folder, camera_id in cameras:
            #     print(f'Saving image from camera {camera_id} to {folder}')
            #     camera.listen(lambda image, f=folder, cid=camera_id: 
            #         save_image(image, f, frame, cid, "rgb" if "rgb" in f else "depth"))
            
            print(f'frame: {frame} max frame: {max_frame} bootstrap frame: {bootstrap_frame}')
            if base_map_only and frame >= 5:
                break
            if not base_map_only and frame >= max_frame+bootstrap_frame: 
                break
            frame += 1
            time.sleep(0.1)
        print("Letting camera sensors to finish capturing the scene")
        if not base_map_only:
            time.sleep(120)
        else:
            time.sleep(10)
    finally:
        print("Cleaning up spawned actors...")
        #cleanup_all_simulation_actors(world)
        safe_cleanup_with_sensors(world)
        print("Cleanup complete.")

def main():
    # Create an ArgumentParser object
    parser = argparse.ArgumentParser(description="A CARLA simulator client that spawns simulated pedestrians and vehicles and captures frames.")
    # Total number of pedestrain spawn points
    parser.add_argument('-p', '--points', type=int, default=200,
                        help='Total number of pedestrain spawn points, final number of spawned pedestrains with AI controllers will be approximately half')
    # camera FoV e.g., 25, 35, 45
    parser.add_argument('-f', '--fov', type=str, default='45',
                        help='camera field of view e.g., 25, 35, 45')
    # camera height e.g., 1.0, 2.0, 3.0
    parser.add_argument('-c', '--camera_height', type=float, default=1.0,
                        help='camera height from ground e.g., 1.0, 2.0, 3.0')
    # pedestrain walking speed
    parser.add_argument('-s', '--speed', type=str, choices=['slow', 'normal', 'fast'],
                        default='slow', help='Choose pedestrain walking speed.')
    # CARLA world weather choice
    parser.add_argument('-w', '--weather', type=str, choices=['morning', 'afternoon', 'late_afternoon', 'evening'],
                        default='slow', help='Choose weather based on time of day; they have different lighting conditions')
    # Total number of pedestrain spawn points
    parser.add_argument('--fps', type=int, default=30,
                        help='CARLA world tick FPS')
    parser.add_argument('-n', '--num_frames', type=int, default=30,
                        help='Number of frames captured by sensors')
    # Parse the arguments from the command line
    args = parser.parse_args()
    print(f'Simulation time: {args.num_frames/args.fps:.2f} seconds')
    try:
        sim_static(args.points, args.fov, args.camera_height, 
                    args.speed, args.weather, args.fps, args.num_frames)
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

if __name__ == '__main__':
    main()
    
