#TODO
START_END_OPT_PLACE_ID = {'route_sn': [241, 261, '<', 0, 1], 'route_ns': [241, 261, '>', 1, 2], 
                'route_se': [241, 261, '<', 0, 3], 'route_es': [241, 261, '>', 1, 4], 
                'route_sw': [241, 261, '<', 0, 5], 'route_ws': [241, 261, '>', 1, 6], 
                'route_ew': [241, 261, '<', 0, 7], 'route_we': [241, 261, '>', 1, 8], 
                'route_en': [241, 261, '<', 0, 9], 'route_ne': [241, 261, '>', 1, 10], 
                'route_wn': [241, 261, '<', 0, 11], 'route_nw': [241, 261, '>', 1, 12]}

                # start_pos, goal_pos, operator, placement, id

AVAILABLE_ROUTES = ['route_sn', 'route_ns', 'route_we', 'route_ew']

class EgoVehicle(object):
    def __init__(self, vehID, typeID):
        self.vehID = vehID
        self.routeID = np.random.choice(AVAILABLE_ROUTES) # random pick a route ID
        self.typeID = typeID
        self.start_speed = round(np.random.uniform() * 10, 2) # random start speed

        self.start_pos = np.random.randint(low=START_END_OPT[routeID][0], high=START_END_OPT[routeID][1])
        self.goal_pos = START_END_OPT[routeID][1]
        self.opt = START_END_OPT[routeID][2]
        self.placement = START_END_OPT[routeID][3]
        self.route_type = START_END_OPT[routeID][4]

    def reached_goal(self, vehicle_pos):
        return eval(str(vehilce_pos[self.placement]) + self.opt + str(self.goal_pos))