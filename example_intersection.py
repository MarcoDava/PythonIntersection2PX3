from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 24
        island_width = 2
        length = 100


#---------------------------------------------------------------Variables----------------------------------------------------------------------------#
        self.vehicle_rate = 10
        self.v = 17
        self.speed_variance = 2
        self.self_driving_vehicle_proportion = 0 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5
#----------------------------------------------------------------------------------------------------------------------------------------------------#
        #south entrance 0
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) 
        #south straight 1
        self.sim.create_segment((lane_space/2 + island_width/2, intersection_size/2), (lane_space/2 + island_width/2, -intersection_size/2)) 
        #north exit 2
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))
        #south right turn 3
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2), (lane_space/2 + island_width/2, lane_space/2 + island_width/2), (intersection_size/2, lane_space/2 + island_width/2))
        
        # south left turn 4
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2), (lane_space/2 + island_width/2, -lane_space/2 - island_width/2), (-intersection_size/2, -lane_space/2 - island_width/2))
        #east exit 5
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))
        #west exit 6
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))
        

        #entrances 7-9 (east, north, west)
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) 
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) 
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) 
        
        #straights 10-12 (east, north, south)
        self.sim.create_segment((intersection_size/2, -lane_space/2 - island_width/2), (-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_segment((-lane_space/2 - island_width/2, -intersection_size/2), (-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_segment((-intersection_size/2, lane_space/2 + island_width/2), (intersection_size/2, lane_space/2 + island_width/2))
        
        #south exit 13
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))
        
        #right turns 14-16 (east, north, south)
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2), (lane_space/2 + island_width/2, -lane_space/2 - island_width/2), (lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, -intersection_size/2), (-lane_space/2 - island_width/2, -lane_space/2 - island_width/2), (-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2), (-lane_space/2 - island_width/2, lane_space/2 + island_width/2), (-lane_space/2 - island_width/2, intersection_size/2))

        #left turns 17-19 (east, north, south)
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2), (-lane_space/2 - island_width/2, -lane_space/2 - island_width/2), (-lane_space/2 - island_width/2, intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, -intersection_size/2), (-lane_space/2 - island_width/2, lane_space/2 + island_width/2), (intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2), (lane_space/2 + island_width/2, lane_space/2 + island_width/2), (lane_space/2 + island_width/2, -intersection_size/2))
        
        self.vg = VehicleGenerator({
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at

            'vehicles': [

                (1, {'path': [0, 1, 2], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [0, 3, 5], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [0, 4, 6], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                (1, {'path': [7, 10, 6], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [7, 14, 2], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [7, 17, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                (1, {'path': [8, 11, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [8, 15, 6], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [8, 18, 5], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                (1, {'path': [9, 12, 5], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [9, 16, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [9, 19, 2], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance})
                ], 'vehicle_rate' : self.vehicle_rate*(1-self.self_driving_vehicle_proportion) 
            })
        
        self.sdvg = VehicleGenerator({
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at

            'vehicles': [
                
                (1, {'path': [0, 1, 2], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [0, 3, 5], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [0, 4, 6], 'v_max': self.v, 'T':0.1,'s0' : 4}),

                (1, {'path': [7, 10, 6], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [7, 14, 2], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [7, 17, 13], 'v_max': self.v, 'T':0.1,'s0' : 4}),

                (1, {'path': [8, 11, 13], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [8, 15, 6], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [8, 18, 5], 'v_max': self.v, 'T':0.1,'s0' : 4}),

                (1, {'path': [9, 12, 5], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [9, 16, 13], 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': [9, 19, 2], 'v_max': self.v, 'T':0.1,'s0' : 4})
                
                
                ], 'vehicle_rate' : self.vehicle_rate*(self.self_driving_vehicle_proportion) 
            })
        self.sig = TrafficSignal([
            [self.sim.segments[0],self.sim.segments[8]],[self.sim.segments[7],self.sim.segments[9]]]
            ,{'cycle':[(False, True,30), (True, False,60)]})

        self.sim.add_traffic_signal(self.sig)

        self.sim.add_vehicle_generator(self.vg)
        self.sim.add_vehicle_generator(self.sdvg)

    def get_sim(self):
        return self.sim