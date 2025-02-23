from trafficSimulator import *
import numpy as np


class AIntersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 49
        island_width = 10#increased gap between roads for the underground passage
        length = 43.75 #I have shortened the length of the entrance roads because vehicles base speeds are much lower because they are driving in a round about, however they would be able to drive faster in the entrance.
        radius = 20

        self.v = 8.5

        #entrance 0-3
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2))#south east
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) #north east
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) #north west
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) #south west
        #exit4-7
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))#south west
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))#south east
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))#north east
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))#north west
        #corners 8-11
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))
        #connectors 12-15
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius))
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2))# connectors are the connections within the roundabout that connect the corners
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius))
        #turn into corners 16-19
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius))#the slight curve into the roundabout
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2))
        #turn to exit 20-23
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2))#the slight curve out of the roundabout
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2))
        #underground passage
        self.sim.create_segment((0, radius*2),(0, radius/2))
        self.sim.create_segment((-radius*2,0),(-radius/2, 0))
        self.sim.create_segment((radius/2,-radius/2),(radius*3/2,-radius*3/2))
        #underground turns to exit
        self.sim.create_quadratic_bezier_curve((0, radius/2),(radius/4,-radius/4),(radius/2,-radius/2))
        self.sim.create_quadratic_bezier_curve((-radius/2, 0),(radius/4,-radius/4),(radius/2,-radius/2))
        #turn into the underground passage
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, length + intersection_size/2),(0,(length + intersection_size/2)/2),(0,radius*2))

        


        self.vg = VehicleGenerator({


            'vehicles': [
                (1, {'path': [0, 16, 8,20,5],'v_max':self.v}),
                (1, {'path': [0, 16, 8,12,9,21,6],'v_max':self.v}),
                (1, {'path': [0, 16, 8,12,9,13,10,22,7],'v_max':self.v}),
                (1, {'path': [0, 16, 8,12,9,13,10,14,11,23,4],'v_max':self.v}),

                (1,{'path': [1, 17, 9, 21, 6],'v_max':self.v}),
                (1, {'path': [1,17,9,13,10,22,7],'v_max':self.v}),
                (1, {'path': [1, 17, 9,13,10,14,11,23,4],'v_max':self.v}),
                (1, {'path': [1, 17, 9,13,10,14,11,15,8,20,5],'v_max':self.v}),

                (1, {'path': [2, 18, 10, 22, 7],'v_max':self.v}),
                (1, {'path': [2,18,10,14,11,23,4],'v_max':self.v}),
                (1, {'path': [2,18,10,14,11,15,8,20,5],'v_max':self.v}),
                (1, {'path': [2, 18, 10,14,11,15,8,12,9,21,6],'v_max':self.v}),
                
                (1, {'path': [3, 19, 11, 23, 4],'v_max':self.v}),
                (1, {'path': [3,19,11,15,8,20,5],'v_max':self.v}),
                (1, {'path': [3,19,11,15,8,12,9,21,6],'v_max':self.v}),
                (1, {'path': [3, 19, 11,15,8,12,9,13,10,22,7],'v_max':self.v}),

            ], 'vehicle_rate': 30
        }
        
        )
        self.sim.define_interfearing_paths([0,16],[15,8],turn=True)
        self.sim.define_interfearing_paths([1,17],[12,9],turn=True)
        self.sim.define_interfearing_paths([2,18],[13,10],turn=True)
        self.sim.define_interfearing_paths([3,19],[14,11],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
    def get_sim(self):
        return self.sim