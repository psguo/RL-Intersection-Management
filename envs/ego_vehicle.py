#TODO
START_END_OPT_PLACE = {'route_sn': [241, 261, '<', 0], 'route_ns': [241, 261, '>', 1], 
                'route_se': [241, 261, '<', 0], 'route_es': [241, 261, '>', 1], 
                'route_sw': [241, 261, '<', 0], 'route_ws': [241, 261, '>', 1], 
                'route_ew': [241, 261, '<', 0], 'route_we': [241, 261, '>', 1], 
                'route_en': [241, 261, '<', 0], 'route_ne': [241, 261, '>', 1], 
                'route_wn': [241, 261, '<', 0], 'route_nw': [241, 261, '>', 1]}

AVAILABLE_ROUTES = ['route_sn', 'route_ns', 'route_we', 'route_ew']

class EgoVehicle(object):
    def __init__(self, vehID, typeID):
        self.vehID = vehID
        self.routeID = np.random.choice(AVAILABLE_ROUTES) # random pick a route ID
        self.typeID = typeID
        self.start_speed = round(np.random.uniform() * 10, 2) # random start speed

        self.start_pos = START_END_OPT[routeID][0]
        self.goal_pos = START_END_OPT[routeID][1]
        self.opt = START_END_OPT[routeID][2]
        self.placement = START_END_OPT[routeID][3]

    def reached_goal(self, vehicle_pos):
        return eval(str(vehilce_pos[self.placement]) + self.opt + str(self.goal_pos))