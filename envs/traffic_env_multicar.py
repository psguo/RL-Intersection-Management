from gym import Env
from gym import error, spaces, utils
from gym.utils import seeding
import traci
import traci.constants as tc
from scipy.misc import imread
from scipy.ndimage import rotate
# from scipy.misc import imsave
import matplotlib.pyplot as plt
from ego_vehicle import EgoVehicle
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


class TrafficEnvMulticar(Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, mode="gui"):
        # "--end", str(simulation_end),
        self.simulation_end = 3600
        self.sleep_between_restart = 0.1
        self.mode = mode
        self._seed()
        self.loops = ["loop{}".format(i) for i in range(12)]
        basepath = os.path.join(os.path.dirname(__file__), "config", "cross")
        netfile = os.path.join(basepath, "cross.net.xml")
        routefile = os.path.join(basepath, "cross.rou.xml")
        guifile = os.path.join(basepath, "view.settings.xml")
        addfile = os.path.join(basepath, "cross.add.xml")
        self.exitloops = ["loop4", "loop5", "loop6", "loop7"]
        self.loop_variables = [tc.LAST_STEP_MEAN_SPEED, tc.LAST_STEP_TIME_SINCE_DETECTION, tc.LAST_STEP_VEHICLE_NUMBER]
        self.lanes = ["n_0_0", "s_0_0", "e_0_0", "w_0_0", "0_n_0", "0_s_0", "0_e_0", "0_w_0"]
        self.detector = "detector0"
        self.tmpfile = "tmp.rou.xml"
        self.pngfile = "tmp.png"
        step_length = "0.1"
        args = ["--net-file", netfile, "--route-files", self.tmpfile, "--additional-files", addfile, "--step-length", step_length]
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
        self.lights = []

        self.action_space = spaces.Discrete(3)
        self.throttle_actions = {0: 0., 1: 1., 2:-1.}

        # self.ego_vehicles = [EgoVehicle('ego_car', 'route_sn', 'EgoCar', 245., 261., 0.),
        #                 EgoVehicle('ego_car', 'route_se', 'EgoCar', 245., 261., 0.),
        #                 EgoVehicle('ego_car', 'route_sw', 'EgoCar', 245., 241., 0.)]
        
        self.ego_vehicles = [EgoVehicle('ego_car_' + (str(i)), 'EgoCar') for i in range(20)]

        self.ego_veh = self.ego_vehicles[0]
        self.ego_veh_collision = False

        self.braking_time = 0.

        # TO DO: re-define observation space !!
        # trafficspace = spaces.Box(low=float('-inf'), high=float('inf'),
        #                           shape=(len(self.loops) * len(self.loop_variables),))
        # lightspaces = [spaces.Discrete(len(light.actions)) for light in self.lights]
        # self.observation_space = spaces.Tuple([trafficspace] + lightspaces)

        self.sumo_running = False
        self.viewer = None


    def route_sample(self):
        # if self.np_random.uniform(0, 1) > 0.5:
        ew = np.random.normal(0.15, 0.02)
        we = np.random.normal(0.12, 0.02)
        ns = np.random.normal(0.08, 0.02)
        sn = 0.01

        routes = {"ns": ns, "sn": sn, "ew": ew, "we": we}

        return routes

    def relative_path(self, *paths):
        os.path.join(os.path.dirname(__file__), *paths)

    def write_routes(self):
        self.route_info = self.route_sample()
        with open(self.tmpfile, 'w') as f:
            f.write(Template(self.route).substitute(self.route_info))

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def start_sumo(self):
        self.route_info = self.route_sample()
        if not self.sumo_running:
            self.write_routes()
            traci.start(self.sumo_cmd)
            for loopid in self.loops:
                traci.inductionloop.subscribe(loopid, self.loop_variables)
            self.sumo_running = True
        else: # Reset vehicles in simulation
            if self.ego_veh.vehID in traci.vehicle.getIDList():
                traci.vehicle.remove(vehID=self.ego_veh.vehID, reason=2)
            traci.simulation.clearPending()

        self.sumo_step = 0
        self.sumo_deltaT = traci.simulation.getDeltaT()/1000. # Simulation timestep in seconds
        for i in range(800):
            traci.simulationStep()
        self.ego_veh = random.choice(self.ego_vehicles)
        self.ego_veh_collision = False
        self.braking_time = 0.
        traci.vehicle.add(vehID=self.ego_veh.vehID, routeID=self.ego_veh.routeID,
                          pos=self.ego_veh.start_pos, speed=self.ego_veh.start_speed, typeID=self.ego_veh.typeID)
        traci.vehicle.setSpeedMode(vehID=self.ego_veh.vehID, sm=0) # All speed checks are off
        # self.screenshot()

    def _stop_sumo(self):
        if self.sumo_running:
            traci.close()
            self.sumo_running = False

    def check_collision(self, ego_vehicle):
        min_dist = 100.00
        ego_pos = np.array(traci.vehicle.getPosition(ego_vehicle.vehID))
        ego_veh_collision = False

        for i in traci.vehicle.getIDList():
            # excluding ego vehicle AND any vehicle from the opposite direction (NS) for comparison
            if i != ego_vehicle.vehID:
                pos = np.array(traci.vehicle.getPosition(i))
                new_dist = np.linalg.norm(ego_pos - pos)
                if new_dist < min_dist:
                    min_dist = new_dist
        
        if min_dist < 1.25: 
            ego_veh_collision = True

        return ego_veh_collision

    # choose action based on the Time-To-Collision metric
    def action_from_ttc(self):
        ego_id = self.ego_veh.vehID
        ego_ang = traci.vehicle.getAngle(ego_id) * math.pi / 180.0 # in radian (clockwise)
        ego_vel = traci.vehicle.getSpeed(ego_id)  # in m/s
        # NOTE different coordinate system for angle and pose!
        ego_vel_x = ego_vel * math.sin(ego_ang) 
        ego_vel_y = ego_vel * math.cos(ego_ang)
        # calculate position of vehicle center instead of front bumper
        ego_pos = np.array(traci.vehicle.getPosition(ego_id))
        # ego_pos[0] = ego_pos[0] - 2.5*math.cos(ego_ang)
        # ego_pos[1] = ego_pos[1] - 2.5*math.sin(ego_ang)

        # initialize 
        min_ttc = 10
        action  = -1
        front_warning = False
        pre_collision = False

        routeID = self.ego_veh.routeID
        # straight cross
        if routeID == 'route_sn':
            for i in traci.vehicle.getIDList(): 
                # excluding ego vehicle AND NS traffic for comparison 
                if i != ego_id and i.find('flow_n_s') == -1:
                    new_ang = traci.vehicle.getAngle(i) * math.pi / 180.0 # in radian (clockwise)
                    new_vel = traci.vehicle.getSpeed(i) # in m/s
                    new_vel_x = new_vel * math.sin(new_ang)
                    new_vel_y = new_vel * math.cos(new_ang)
                    new_pos = np.array(traci.vehicle.getPosition(i)) # in m

                    rel_vel = np.linalg.norm([ego_vel_x - new_vel_x, ego_vel_y - new_vel_y])
                    rel_dist = np.linalg.norm(ego_pos - new_pos)
                    new_ttc = rel_dist / rel_vel

                    rel_ang = math.atan2(new_pos[0] - ego_pos[0], new_pos[1] - ego_pos[1]) # in radian
                    if rel_ang < 0.0:
                        rel_ang = 2*math.pi + rel_ang

                    if abs(rel_ang - ego_ang) < 0.5 and rel_dist < 10.0: 
                        front_warning = True
                        if rel_dist < 2.0:
                            pre_collision = True

                    if new_ttc < min_ttc:
                        min_ttc = new_ttc
        # left turn 
        elif routeID == 'route_sw':
            for i in traci.vehicle.getIDList():
                # excluding ego vehicle for comparison only
                if i != ego_id:
                    new_ang = traci.vehicle.getAngle(i) * math.pi / 180.0 # in radian (clockwise)
                    new_vel = traci.vehicle.getSpeed(i) # in m/s
                    new_vel_x = new_vel * math.sin(new_ang)
                    new_vel_y = new_vel * math.cos(new_ang)
                    new_pos = np.array(traci.vehicle.getPosition(i)) # in m

                    rel_vel = np.linalg.norm([ego_vel_x - new_vel_x, ego_vel_y - new_vel_y])
                    rel_dist = np.linalg.norm(ego_pos - new_pos)
                    new_ttc = rel_dist / rel_vel

                    rel_ang = math.atan2(new_pos[0] - ego_pos[0], new_pos[1] - ego_pos[1]) # in radian
                    if rel_ang < 0.0:
                        rel_ang = 2*math.pi + rel_ang

                    if abs(rel_ang - ego_ang) < 0.5 and rel_dist < 10.0: 
                        front_warning = True
                        if rel_dist < 2.0:
                            pre_collision = True

                    if new_ttc < min_ttc:
                        min_ttc = new_ttc

        # right turn 
        elif routeID == 'route_se':
            for i in traci.vehicle.getIDList():
                # ONLY take WE traffic for comparison 
                if i.find('flow_w_e') != -1: 
                    new_ang = traci.vehicle.getAngle(i) * math.pi / 180.0 # in radian (clockwise)
                    new_vel = traci.vehicle.getSpeed(i) # in m/s
                    new_vel_x = new_vel * math.sin(new_ang)
                    new_vel_y = new_vel * math.cos(new_ang)
                    new_pos = np.array(traci.vehicle.getPosition(i)) # in m

                    rel_vel = np.linalg.norm([ego_vel_x - new_vel_x, ego_vel_y - new_vel_y])
                    rel_dist = np.linalg.norm(ego_pos - new_pos)
                    new_ttc = rel_dist / rel_vel

                    rel_ang = math.atan2(new_pos[0] - ego_pos[0], new_pos[1] - ego_pos[1]) # in radian
                    if rel_ang < 0.0:
                        rel_ang = 2*math.pi + rel_ang

                    if abs(rel_ang - ego_ang) < 0.5 and rel_dist < 10.0: 
                        front_warning = True
                        if rel_dist < 2.0:
                            pre_collision = True

                    if new_ttc < min_ttc:
                        min_ttc = new_ttc

        else: 
            print 'Invalid route for ego-vehicle. No action will be taken.'

        # decision making based on min time-to-collision
        if (front_warning and min_ttc < 3.0) or pre_collision: 
            action = 2
        else:
            if min_ttc > 2.0 or abs(ego_pos[0] - 250) > 3 or ego_pos[1] - 250 > 2:
                action = 1
            elif min_ttc < 1.5:
                action = 2
            else: 
                action = 0 

        # print 'min ttc: ', min_ttc, ' front warning: ', front_warning

        return action

    # TODO: Refine reward function!!
    def _reward(self):
        reward = 0
        self.is_done = False
        for ego_vehicle in self.ego_vehicles:
            pos = traci.vehicle.getPosition(vehicle_id)
            if ego_vehicle.reached_goal(pos):
                reward += 1000
                self.is_done = True
            if self.check_collision(ego_vehicle):
                reward -= 50000
                self.is_done = True

        reward -= 1
        return reward

    def _step(self, action):
        if not self.sumo_running:
            self.start_sumo()
        self.sumo_step += 1

        # if action == 2:
        #     self.braking_time += 1


        for i, vehicle in enumerate(self.ego_vehicles):
            new_speed = traci.vehicle.getSpeed(vehicle.vehID) + self.sumo_deltaT * self.throttle_actions[action[i]]
            traci.vehicle.setSpeed(vehicle.vehID, new_speed)

        # print("Step = ", self.sumo_step, "   | action = ", action)
        # print("car speed = ", traci.vehicle.getSpeed(self.ego_veh.vehID), "   | new speed = ",new_speed)

        traci.simulationStep()
        observation = self._observation()
        reward = self._reward()

        # print self.check_collision()

        done = self.is_done \
               or (self.sumo_step > self.simulation_end) \
               # or (self.ego_veh.vehID not in traci.vehicle.getIDList()) \
        # self.screenshot()
        # if done:
        #     print "Collision?  ", self.ego_veh_collision
        #     print "Steps = ", self.sumo_step, "      |    braking steps = ", self.braking_time
        
        return observation, reward, done, self.route_info

    def screenshot(self):
        if self.mode == "gui":
            traci.gui.screenshot("View #0", self.pngfile)

    def _observation(self):
        state = []

        for ego_vehicle in self.ego_vehicles:
            vehicle_id = ego_vehicle.vehID
            pos = traci.vehicle.getPosition(vehicle_id)
            speed = traci.vehicle.getSpeed(vehicle_id)
            angle = traci.vehicle.getAngle(vehicle_id)

            state += [pos[0], pos[1], speed, angle, ego_vehicle.route_type]

        return state

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
