from gym import Env
from gym import error, spaces, utils
from gym.utils import seeding
import traci
import traci.constants as tc
from scipy.misc import imread
from scipy.ndimage import rotate
# from scipy.misc import imsave
import matplotlib.pyplot as plt
from envs.ego_vehicle import EgoVehicle
from gym import spaces
from string import Template
import os, sys
import numpy as np
import math
import random
import time

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

AVAILABLE_ROUTES = ['route_sn', 'route_ns', 'route_we', 'route_ew']
NUM_VEHICLES = 16
IS_TEST = False

class TrafficEnvMulticar(Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, mode="gui"):
        # "--end", str(simulation_end),
        self.simulation_end = 3600
        self.sleep_between_restart = 0.1
        self.mode = mode
        self._seed()
        # self.loops = ["loop{}".format(i) for i in range(12)]
        basepath = os.path.join(os.path.dirname(__file__), "config", "multicar")
        netfile = os.path.join(basepath, "multicar.net.xml")
        routefile = os.path.join(basepath, "multicar.rou.xml")
        guifile = os.path.join(basepath, "view.settings.xml")
        # addfile = os.path.join(basepath, "multicar.add.xml")
        # self.exitloops = ["loop4", "loop5", "loop6", "loop7"]
        # self.loop_variables = [tc.LAST_STEP_MEAN_SPEED, tc.LAST_STEP_TIME_SINCE_DETECTION, tc.LAST_STEP_VEHICLE_NUMBER]
        # self.lanes = ["n_0_0", "s_0_0", "e_0_0", "w_0_0", "0_n_0", "0_s_0", "0_e_0", "0_w_0"]
        # self.detector = "detector0"
        self.tmpfile = "tmp.rou.xml"
        self.pngfile = "tmp.png"
        step_length = "0.1"
        # args = ["--net-file", netfile, "--route-files", routefile, "--additional-files", addfile, "--step-length", step_length]
        args = ["--net-file", netfile, "--route-files", routefile, "--step-length", step_length, "--no-warnings", "--collision.check-junctions", "true", "--collision.action", "remove", ]
                # "--collision.check-junctions", "true", "--collision.action", "remove", "--no-warnings"]

        if mode == "gui":
            binary = "sumo-gui"
            args += ["-S", "-Q", "--gui-settings-file", guifile]
        else:
            binary = "sumo"
            args += ["--no-step-log"]

        with open(routefile) as f:
            self.route = f.read()
        self.sumo_cmd = [binary] + args
        self.sumo_step = 0
        # self.lights = []

        self.max_accleration=1.

        self.action_space = spaces.Box(low=-self.max_accleration, high=self.max_accleration, shape=(NUM_VEHICLES,))
        self.observation_space = spaces.Box(low=float('-inf'), high=float('inf'), shape=(NUM_VEHICLES*3,))

        self.state = np.zeros((NUM_VEHICLES, 3))

        # self.throttle_actions = {0: 0., 1: 1., 2:-1.}

        # self.ego_vehicles = [EgoVehicle('ego_car', 'route_sn', 'EgoCar', 245., 261., 0.),
        #                 EgoVehicle('ego_car', 'route_se', 'EgoCar', 245., 261., 0.),
        #                 EgoVehicle('ego_car', 'route_sw', 'EgoCar', 245., 241., 0.)]
        
        # self.collision_thresh = 15
        # self.init_vehicles()

        # self.ego_veh = self.ego_vehicles[0]

        self.braking_time = 0.

        # TO DO: re-define observation space !!
        # trafficspace = spaces.Box(low=float('-inf'), high=float('inf'),
        #                           shape=(len(self.loops) * len(self.loop_variables),))
        # lightspaces = [spaces.Discrete(len(light.actions)) for light in self.lights]

        self.sumo_running = False
        self.viewer = None

    def relative_path(self, *paths):
        os.path.join(os.path.dirname(__file__), *paths)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def init_vehicles(self):
        self.ego_vehicles = {}
        for i in range(NUM_VEHICLES):
            vehID = str(i)
            routeID = AVAILABLE_ROUTES[i%4]
            vehicle_candidate = self.init_vehicle(vehicle_id=vehID, route_id=routeID)
            self.ego_vehicles[vehID] = vehicle_candidate
        # self.init_vehicles_speed()
        # import IPython
        # IPython.embed()
    def init_vehicles_speed(self):
        pass

        # import IPython
        # IPython.embed()

    # def init_vehicle(self, vehicle_id):
    #     while True:s
    #         vehicle_candidate = EgoVehicle(vehicle_id, 'EgoCar')
    #         for exist_vehicle in self.ego_vehicles.values():
    #             if np.linalg.norm(exist_vehicle.start_pos - vehicle_candidate.start_pos) <= self.collision_thresh:
    #                 continue
    #         break
    #     return vehicle_candidate

    def init_vehicle(self, vehicle_id, route_id=None):
        while True:
            # import IPython
            # IPython.embed()
            # print(traci.vehicle.getIDList())
            vehicle_candidate = EgoVehicle(vehicle_id, 'EgoCar', routeID=route_id)
            traci.vehicle.add(vehID=vehicle_id, routeID=vehicle_candidate.routeID,
                              pos=vehicle_candidate.start_pos, speed=vehicle_candidate.start_speed, typeID=vehicle_candidate.typeID)
            traci.vehicle.setSpeedMode(vehID=vehicle_id, sm=0) # All speed checks are off
            traci.vehicle.setSpeed(vehicle_id, 0)

            traci.simulationStep()

            if traci.vehicle.getPosition(vehicle_id) != (-1001., -1001.):
                break

            traci.vehicle.remove(vehID=vehicle_id, reason=2)

            # print("--------------------------")
            # print(vehicle_id)
            # print(vehicle_candidate.start_pos)
            # print("--------------------------")

        return vehicle_candidate

    def reset_vehicle(self, vehicle_id, route_id=None):

        # import IPython
        # IPython.embed()
        # print(traci.vehicle.getIDList())
        
        vehicle_speed = traci.vehicle.getSpeed(vehicle_id)
        traci.vehicle.remove(vehicle_id)
        vehicle_candidate = EgoVehicle(vehicle_id, 'EgoCar', routeID=route_id, random_start=False)
        traci.vehicle.add(vehID=vehicle_id, routeID=vehicle_candidate.routeID,
                          pos=vehicle_candidate.start_pos, speed=vehicle_candidate.start_speed, typeID=vehicle_candidate.typeID)
        traci.vehicle.setSpeedMode(vehID=vehicle_id, sm=0) # All speed checks are off
        traci.vehicle.setSpeed(vehicle_id, vehicle_speed)

        # traci.simulationStep()

        # if traci.vehicle.getPosition(vehicle_id) != (-1001., -1001.):
        #     return vehicle_candidate, False

        # traci.vehicle.remove(vehID=vehicle_id, reason=2)

        # print("--------------------------")
        # print(vehicle_id)
        # print(vehicle_candidate.start_pos)
        # print("--------------------------")

        return vehicle_candidate

    def start_sumo(self):
        if not self.sumo_running:
            traci.start(self.sumo_cmd)
            # for loopid in self.loops:
                # traci.inductionloop.subscribe(loopid, self.loop_variables)
            self.sumo_running = True
        else: # Reset vehicles in simulation
            for i in traci.vehicle.getIDList():
                traci.vehicle.remove(vehID=i, reason=2)
            traci.simulation.clearPending()

        self.sumo_step = 0
        self.sumo_deltaT = traci.simulation.getDeltaT()/1000. # Simulation timestep in seconds
        
        self.braking_time = 0.

        # import IPython
        # IPython.embed()
        self.init_vehicles()

        # print(self.ego_vehicles)

        # for ego_vehicle in self.ego_vehicles.values():
        #     # print(ego_vehicle.start_pos)
        #     # import IPython
        #     # IPython.embed()

        #     traci.vehicle.add(vehID=ego_vehicle.vehID, routeID=ego_vehicle.routeID,
        #                       pos=ego_vehicle.start_pos, speed=ego_vehicle.start_speed, typeID=ego_vehicle.typeID)
        #     traci.vehicle.setSpeedMode(vehID=ego_vehicle.vehID, sm=0) # All speed checks are off
        #     traci.vehicle.setSpeed(ego_vehicle.vehID, 0)
            # traci.simulationStep()

        # self.screenshot()


        traci.simulationStep()
        # import IPython
        # IPython.embed()

    def _stop_sumo(self):
        if self.sumo_running:
            traci.close()
            self.sumo_running = False

    def location2bounds(self, x, y, angle, bound):
        car_length = 5 # meters
        car_width = 1.8 # meters
        # continuous bounds
        car_c_bound_x_1 = 0
        car_c_bound_x_2 = 0
        car_c_bound_y_1 = 0
        car_c_bound_y_2 = 0
        # if orientation == 'vertical':
        #     car_c_bound_x_1 = x-(car_width/2.0)
        #     car_c_bound_x_2 = x+(car_width/2.0)
        #     car_c_bound_y_1 = y-(car_length/2.0)
        #     car_c_bound_y_2 = y+(car_length/2.0)
        # elif orientation == 'horizontal':
        #     car_c_bound_x_1 = x-(car_length/2.0)
        #     car_c_bound_x_2 = x+(car_length/2.0)
        #     car_c_bound_y_1 = y-(car_width/2.0)
        #     car_c_bound_y_2 = y+(car_width/2.0)
        if (abs(angle - 0.0) < 0.01):
            car_c_bound_x_1 = x-(car_width/2.0)
            car_c_bound_x_2 = x+(car_width/2.0)
            car_c_bound_y_1 = y-car_length
            car_c_bound_y_2 = y
        elif (abs(angle - 180.0) < 0.01):
            car_c_bound_x_1 = x-(car_width/2.0)
            car_c_bound_x_2 = x+(car_width/2.0)
            car_c_bound_y_1 = y
            car_c_bound_y_2 = y+car_length
        elif (abs(angle - 90.0) < 0.01):
            car_c_bound_x_1 = x-car_length
            car_c_bound_x_2 = x
            car_c_bound_y_1 = y-(car_width/2.0)
            car_c_bound_y_2 = y+(car_width/2.0)
        elif (abs(angle - 270.0) < 0.01):
            car_c_bound_x_1 = x
            car_c_bound_x_2 = x+car_length
            car_c_bound_y_1 = y-(car_width/2.0)
            car_c_bound_y_2 = y+(car_width/2.0)


        # discrete bounds
        car_d_bound_x_1 = np.floor(car_c_bound_x_1)+np.floor(bound/2.0)
        car_d_bound_x_2 = np.floor(car_c_bound_x_2)+np.floor(bound/2.0)
        car_d_bound_y_1 = np.floor(car_c_bound_y_1)+np.floor(bound/2.0)
        car_d_bound_y_2 = np.floor(car_c_bound_y_2)+np.floor(bound/2.0)

        if (car_d_bound_x_1 < 0):
            car_d_bound_x_1 = 0
        if (car_d_bound_x_2 < 0):
            car_d_bound_x_2 = 0
        if (car_d_bound_y_1 < 0):
            car_d_bound_y_1 = 0
        if (car_d_bound_y_2 < 0):
            car_d_bound_y_2 = 0
        if (car_d_bound_x_1 >= bound):
            car_d_bound_x_1 = bound-1
        if (car_d_bound_x_2 >= bound):
            car_d_bound_x_2 = bound-1
        if (car_d_bound_y_1 >= bound):
            car_d_bound_y_1 = bound-1
        if (car_d_bound_y_2 >= bound):
            car_d_bound_y_2 = bound-1

        return np.array([car_d_bound_x_1, car_d_bound_x_2, car_d_bound_y_1, car_d_bound_y_2]).astype(int)

    def render_image(self):
        intersection_cars = []
        center_coord = [100, 100]
        bound = 20

        for i in traci.vehicle.getIDList():
            speed = traci.vehicle.getSpeed(i)
            pos = traci.vehicle.getPosition(i)
            angle = traci.vehicle.getAngle(i)
            laneid = traci.vehicle.getRouteID(i)
            state_tuple = (i, pos[0], pos[1], angle, speed, laneid)

            if(np.linalg.norm(np.asarray(pos)-np.asarray(center_coord))<bound/2):
                intersection_cars.append(state_tuple)

        obstacle_image = np.zeros((bound,bound,len(intersection_cars)))

        if len(intersection_cars) == 0:
            return obstacle_image

        for i, vehicle_info in enumerate(intersection_cars):

            car_bounds = self.location2bounds(vehicle_info[1]-center_coord[0], vehicle_info[2]-center_coord[1], vehicle_info[3], bound)
            obstacle_image[bound-1-car_bounds[3]:bound-car_bounds[2], car_bounds[0]:car_bounds[1]+1, i] = 1

            # obstacle_image[:,:,i] = (np.clip(obstacle_image[:,:,i], 0, 1))

        # print (obstacle_image.shape)
        show_image = np.sum(obstacle_image, axis=2)

        # plt.imsave('test.jpg', obstacle_image)
        # plt.ion()
        # plt.imshow(obstacle_image)
        # plt.imshow(show_image)
        # plt.imshow(obstacle_image[:,:,1])
        # plt.imshow(obstacle_image[:,:,2])
        # plt.draw(plt.imshow(obstacle_image))
        # plt.draw()
        # time.sleep(1.0)
        # time.sleep(5.0)
        # plt.show()
        # import IPython
        # IPython.embed()

        return obstacle_image


    def check_collision(self):

        # import IPython
        # IPython.embed()

        ego_veh_collision = False

        # 0. Collision: collision after resetting
        # for vehicle_id in self.ego_vehicles:
        #     if traci.vehicle.getPosition(vehicle_id) == (-1001., -1001.):
        #         if IS_TEST:
        #             print("Reset collision")
        #         ego_veh_collision = True
        #         return ego_veh_collision

        # 1. Collision: different lane, image
        # obstacle_image = self.render_image()

        # if (np.sum(obstacle_image, axis=2) > 1).any():
        #     if IS_TEST:
        #         print("Intersection collision")
        #     ego_veh_collision = True
        #     return ego_veh_collision

        # 2. Collsion: same lane teleported
        teleported_cars = traci.simulation.getStartingTeleportIDList()

        # if len(teleported_cars) > 0:
        #     if IS_TEST:
        #         print("Teleported collision: ", str(teleported_cars))
        #     ego_veh_collision = True
            # for vehicle_id in teleported_cars:
            #     if vehicle_id in traci.vehicle.getIDList():
            #         traci.vehicle.remove(vehID=vehicle_id, reason=2)
        # print("collision detected")


        if len(traci.vehicle.getIDList()) < NUM_VEHICLES:
            ego_veh_collision = True


        return ego_veh_collision

    # TODO: Refine reward function!!
    def _reward(self):
        reward = 0
        self.is_done = False
        
        self.vehicles_reached_goal = []

        sum_speed = 0

        for vehicle_id in traci.vehicle.getIDList():
            sum_speed += traci.vehicle.getSpeed(vehicle_id)

        reward = sum_speed

        if self.check_collision():
            reward -= 100
            self.is_done = True
        
        # reward -= 1
        return reward

    def _step(self, action):
        if not self.sumo_running:
            self.start_sumo()
        self.sumo_step += 1

        # if action == 2:
        #     self.braking_time += 1

        for ego_vehicle in self.ego_vehicles.values():
            index = int(ego_vehicle.vehID)
            new_speed = traci.vehicle.getSpeed(ego_vehicle.vehID) + self.sumo_deltaT * action[index]
            new_speed = max(new_speed, 0)

            # print("---------------")
            # print(ego_vehicle.vehID)
            # print(new_speed)
            # print(action[index])
            # print("---------------")

            traci.vehicle.setSpeed(ego_vehicle.vehID, new_speed)

        # print("Step = ", self.sumo_step, "   | action = ", action)
        # print("car speed = ", traci.vehicle.getSpeed(self.ego_veh.vehID), "   | new speed = ",new_speed)

        traci.simulationStep()



        observation = self._observation()
        reward = self._reward()

        # print self.check_collision()


        # reset positions of vehicle that reaches goal
        for vehicle_id in traci.vehicle.getIDList():
            ego_vehicle = self.ego_vehicles[vehicle_id]
            pos = traci.vehicle.getPosition(vehicle_id)

            if ego_vehicle.reached_goal(pos):
                # reward += 1000
                vehicle_candidate = self.reset_vehicle(vehicle_id=vehicle_id, route_id=self.ego_vehicles[vehicle_id].routeID)
                self.ego_vehicles[vehicle_id] = vehicle_candidate
            
            if IS_TEST:
                print("Reached Goal: " + str(vehicle_id))

        done = self.is_done
        # done = self.is_done or (self.sumo_step > self.simulation_end)
               # or (self.ego_veh.vehID not in traci.vehicle.getIDList()) \
        # self.screenshot()
        # if done:
        #     print "Collision?  ", self.ego_veh_collision
        #     print "Steps = ", self.sumo_step, "      |    braking steps = ", self.braking_time
        # if self.sumo_step == 1000:
        #     print("========================================")
        
        # if done:
        #     print('------------------------------------------')
        #     print("done: " + str(self.sumo_step))

        #   str([reward, done, [self.sumo_step]]))

        return observation, reward, done, {'episode':self.sumo_step}

    def screenshot(self):
        if self.mode == "gui":
            traci.gui.screenshot("View #0", self.pngfile)

    def _observation(self):
        for vehicle_id in traci.vehicle.getIDList():
            pos = traci.vehicle.getPosition(vehicle_id)
            speed = traci.vehicle.getSpeed(vehicle_id)
            self.state[int(vehicle_id), :] = np.array([pos[0], pos[1], speed])
        return self.state.flatten()

    def _reset(self):
        # if self.sumo_running:
        #     traci.vehicle.remove(vehID=self.ego_veh.vehID, reason=2)
        #     traci.simulation.clearPending()
        #     self.sumo_step = 0
        #     for i in range(800):
        #         traci.simulationStep()
        #     self.ego_veh = random.choice(self.ego_vehicles)
        #     self.ego_veh_collision = False
        #     traci.vehicle.add(vehID=self.ego_veh.vehID, routeID=self.ego_veh.routeID,
        #                       pos=self.ego_veh.start_pos, speed=self.ego_veh.start_speed, typeID=self.ego_veh.typeID)
        #     traci.vehicle.setSpeedMode(vehID=self.ego_veh.vehID, sm=0) # All speed checks are off
        # else:
        #     # self.stop_sumo()
        #     self.ego_veh = random.choice(self.ego_vehicles)
        #     # sleep required on some systems
        #     if self.sleep_between_restart > 0:
        #         time.sleep(self.sleep_between_restart)
        self.start_sumo()
        observation = self._observation()
        return observation

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return
        if self.mode == "gui":
            img = imread(self.pngfile, mode="RGB")
            if mode == 'rgb_array':
                return img
            elif mode == 'human':
                from gym.envs.classic_control import rendering
                if self.viewer is None:
                    self.viewer = rendering.SimpleImageViewer()
                self.viewer.imshow(img)
        else:
            raise NotImplementedError("Only rendering in GUI mode is supported. Please use Traffic-...-gui-v0.")
