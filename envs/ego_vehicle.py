import numpy as np

#TODO
START_END_OPT_PLACE_ID = {
    'route_sn': [150, 240, 261, '>', 1, 1], 
    'route_ns': [150, 240, 261, '<', 1, 2], 
    'route_ew': [150, 240, 261, '<', 0, 3], 
    'route_we': [150, 240, 261, '>', 0, 4]}

                # init_pos_min, init_pos_max, goal_pos, operator, placement, id

AVAILABLE_ROUTES = ['route_sn', 'route_ns', 'route_we', 'route_ew']

class EgoVehicle(object):
    def __init__(self, vehID, typeID):
        self.vehID = vehID
        self.routeID = np.random.choice(AVAILABLE_ROUTES) # random pick a route ID
        self.typeID = typeID
        # self.start_speed = round(np.random.uniform() * 10, 2) # random start speed

        self.start_speed = 0.

        self.start_pos = np.random.randint(low=START_END_OPT_PLACE_ID[self.routeID][0], high=START_END_OPT_PLACE_ID[self.routeID][1])
        self.goal_pos = START_END_OPT_PLACE_ID[self.routeID][2]
        self.opt = START_END_OPT_PLACE_ID[self.routeID][3]
        self.placement = START_END_OPT_PLACE_ID[self.routeID][4]
        self.route_type = START_END_OPT_PLACE_ID[self.routeID][5]

    def reached_goal(self, vehicle_pos):
        return eval(str(vehicle_pos[self.placement]) + self.opt + str(self.goal_pos))