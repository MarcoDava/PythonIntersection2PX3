from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 49
        island_width = 10
        length = 43.75 #I have shortened the length of the entrance roads because vehicles base speeds are much lower because they are driving in a round about, however they would be able to drive faster in the entrance.
        radius = 20

        self.v = 8.5

        #inner entrance 0-3
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) #south east
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) #north east
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) #north west
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2))#south west

        #inner exit4-7
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))#south west
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))# south east
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))#north east
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))#north west

        #inner corners 8-11
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))

        #inner connectors 12-15
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius))
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius))

        #inner turn into corners 16-19
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2))
        
        #inner turn to exit 20-23
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2))
    
        #outer entrance 24-27
        self.sim.create_segment((lane_space*3/2 + island_width/2, length+intersection_size/2), (lane_space*3/2+island_width/2, intersection_size/2))
        self.sim.create_segment((length + intersection_size/2, -lane_space*3/2 - island_width/2), (intersection_size/2, - lane_space*3/2 - island_width/2))
        self.sim.create_segment((-lane_space*3/2 - island_width/2, -length - intersection_size/2), (-lane_space*3/2 - island_width/2, -intersection_size/2))
        self.sim.create_segment((-length - intersection_size/2, lane_space*3/2 + island_width/2), (-intersection_size/2, lane_space*3/2 + island_width/2))

        #outer exit 28-31
        self.sim.create_segment((-lane_space*3/2 - island_width/2, intersection_size/2), (-lane_space*3/2 - island_width/2, length + intersection_size/2))
        self.sim.create_segment((intersection_size/2, lane_space*3/2 + island_width/2), (length+intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_segment((lane_space*3/2 + island_width/2, -intersection_size/2), (lane_space*3/2 + island_width/2, -length-intersection_size/2))
        self.sim.create_segment((-intersection_size/2, -lane_space*3/2 - island_width/2), (-length - intersection_size/2, -lane_space*3/2 - island_width/2))
        
        #outer corners 32-35
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius + lane_space),(radius + lane_space,radius + lane_space),(radius + lane_space,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius + lane_space,-lane_space - island_width/2),(radius + lane_space,-radius - lane_space),(lane_space + island_width/2,-radius - lane_space))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius - lane_space),(-radius - lane_space,-radius - lane_space),(-radius - lane_space,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius - lane_space,lane_space + island_width/2),(-radius - lane_space,radius + lane_space),(-lane_space - island_width/2, radius + lane_space))

        #outer connectors 36-39
        self.sim.create_segment((radius + lane_space,lane_space + island_width/2),(radius + lane_space,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius - lane_space),(-lane_space - island_width/2,-radius - lane_space))
        self.sim.create_segment((-radius - lane_space,-lane_space - island_width/2),(-radius - lane_space,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius + lane_space),(lane_space + island_width/2, radius + lane_space))

        #outer turn into corners 40-43
        self.sim.create_quadratic_bezier_curve((lane_space*3/2 + island_width/2, intersection_size/2), (lane_space*3/2 + island_width/2, radius + lane_space), (lane_space*2 + island_width/2, radius + lane_space))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space*3/2 - island_width/2), (radius + lane_space, -lane_space*3/2 - island_width/2), (radius + lane_space, -lane_space*2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space*3/2 - island_width/2, - intersection_size/2), (-lane_space*3/2 - island_width/2, -radius - lane_space), (-lane_space*2 - island_width/2, -radius - lane_space))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space*3/2 + island_width/2), (-radius - lane_space, lane_space*3/2 + island_width/2), (-radius - lane_space, lane_space*2 + island_width/2))

        #outer turn to exit 44-47
        self.sim.create_quadratic_bezier_curve((radius + lane_space, lane_space*2 + island_width/2), (radius + lane_space, lane_space*3/2 + island_width/2), (intersection_size/2, lane_space*3/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space*2 + island_width/2, -radius - lane_space), (lane_space*3/2 + island_width/2, -radius - lane_space), (lane_space*3/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius - lane_space, -lane_space*2 - island_width/2), (-radius - lane_space, -lane_space*3/2 - island_width/2), (-intersection_size/2, -lane_space*3/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space*2 - island_width/2, radius + lane_space), (-lane_space*3/2 - island_width/2, radius + lane_space), (-lane_space*3/2 - island_width/2, intersection_size/2))

        #underground passage 48-50
        self.sim.create_segment((0, radius*2),(0, radius/2))
        self.sim.create_segment((-radius*2,0),(-radius/2, 0))
        self.sim.create_segment((radius/2,-radius/2),(radius*3/2,-radius*3/2))
        #underground turns to exit 51-52
        self.sim.create_quadratic_bezier_curve((0, radius/2),(0,0),(radius/2,-radius/2))
        self.sim.create_quadratic_bezier_curve((-radius/2, 0),(0,0),(radius/2,-radius/2))
        #turn into the underground passage 53
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, length + intersection_size/2),(0,(length + intersection_size/2)/2),(0,radius*2))
        self.sim.create_quadratic_bezier_curve((-length - intersection_size/2,lane_space/2 + island_width/2),(-(length + intersection_size/2)/2,0),(-radius*2,0))


        self.vg = VehicleGenerator({


            'vehicles': [
                #(1, {'path': [0, 16, 8,20,5],'v_max':self.v}),# need to delete the correct exit
                (1, {'path': [0, 16, 8,12,9,21,6],'v_max':self.v}),
                (1, {'path': [0, 16, 8,12,9,13,10,22,7],'v_max':self.v}),
                (1, {'path': [0, 16, 8,12,9,13,10,14,11,23,4],'v_max':self.v}),

                #(1,{'path': [1, 17, 9, 21, 6],'v_max':self.v}),
                (1, {'path': [1,17,9,13,10,22,7],'v_max':self.v}),
                (1, {'path': [1, 17, 9,13,10,14,11,23,4],'v_max':self.v}),
                (1, {'path': [1, 17, 9,13,10,14,11,15,8,20,5],'v_max':self.v}),

                #(1, {'path': [2, 18, 10, 22, 7],'v_max':self.v}),
                (1, {'path': [2,18,10,14,11,23,4],'v_max':self.v}),
                (1, {'path': [2,18,10,14,11,15,8,20,5],'v_max':self.v}),
                (1, {'path': [2, 18, 10,14,11,15,8,12,9,21,6],'v_max':self.v}),
                
                #(1, {'path': [3, 19, 11, 23, 4],'v_max':self.v}),
                (1, {'path': [3,19,11,15,8,20,5],'v_max':self.v}),
                (1, {'path': [3,19,11,15,8,12,9,21,6],'v_max':self.v}),
                (1, {'path': [3, 19, 11,15,8,12,9,13,10,22,7],'v_max':self.v}),

                (1, {'path': [24, 40, 32,44,29],'v_max':self.v}),#outer lane, need to change so they only exit to the correct exit
                # (1, {'path': [24, 40, 32,36,33,45,30],'v_max':self.v}),
                # (1, {'path': [24, 40, 32,36,33,37,34,46,31],'v_max':self.v}),
                # (1, {'path': [24, 40, 32,36,33,37,34,38,35,47,28],'v_max':self.v}),

                (1, {'path': [25, 41, 33, 45, 30],'v_max':self.v}),
                # (1, {'path': [25,41,33,37,34,46,31],'v_max':self.v}),
                # (1, {'path': [25, 41, 33,37,34,38,35,47,28],'v_max':self.v}),
                # (1, {'path': [25, 41, 33,37,34,38,35,39,32,44,29],'v_max':self.v}),

                (1, {'path': [26, 42, 34, 46, 31],'v_max':self.v}),
                # (1, {'path': [26,42,34,38,35,47,28],'v_max':self.v}),
                # (1, {'path': [26,42,34,38,35,39,32,44,29],'v_max':self.v}),
                # (1, {'path': [26, 42, 34,38,35,39,32,36,33,45,30],'v_max':self.v}),

                (1, {'path': [27, 43, 35, 47, 28],'v_max':self.v}),
                # (1, {'path': [27,43,35,39,32,44,29],'v_max':self.v}),
                # (1, {'path': [27,43,35,39,32,36,33,45,30],'v_max':self.v}),
                # (1, {'path': [27, 43, 35,39,32,36,33,37,34,46,31],'v_max':self.v}),
            ], 'vehicle_rate': 30
        }
        
        )
        self.sim.define_interfearing_paths([0,16],[15,8],turn=True)
        self.sim.define_interfearing_paths([1,17],[12,9],turn=True)
        self.sim.define_interfearing_paths([2,18],[13,10],turn=True)
        self.sim.define_interfearing_paths([3,19],[14,11],turn=True)
        self.sim.define_interfearing_paths([24,40],[39,32],turn=True)
        self.sim.define_interfearing_paths([25,41],[36,33],turn=True)
        self.sim.define_interfearing_paths([26,42],[37,34],turn=True)
        self.sim.define_interfearing_paths([27,43],[38,35],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
    def get_sim(self):
        return self.sim