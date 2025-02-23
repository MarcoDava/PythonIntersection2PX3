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

        #inner exit 4-7
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

        #underground passage straights 48-49
        self.sim.create_segment((0, radius*2),(0, radius/2))
        self.sim.create_segment((-radius*2,0),(-radius/2, 0))

        #underground connection 50
        self.sim.create_segment((radius/2,-radius/2),(radius*3/2,-radius*3/2))

        #underground turns to exit 51-52
        self.sim.create_quadratic_bezier_curve((0, radius/2),(0,0),(radius/2,-radius/2))
        self.sim.create_quadratic_bezier_curve((-radius/2, 0),(0,0),(radius/2,-radius/2))
        
        #spawn and entrance into the underground passage 53-54
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2),(0,radius*2))
        self.sim.create_segment((-length - intersection_size/2,lane_space/2 + island_width/2),(-radius*2,0))

        #north west entrance curve into exit 55
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, -length - intersection_size/2),(0,-radius*2),(radius*3/2,-radius*3/2))

        #underground exit 56
        self.sim.create_segment((radius*3/2,-radius*3/2),(intersection_size,-intersection_size))



        self.vg = VehicleGenerator({


            'vehicles': [
                (1, {'path': [0, 16, 8,12,9,21,6],'v_max':self.v, 'colour':(225,225,0)}),  #left lane, bottom to top
                (1, {'path': [0, 16, 8,12,9,13,10,22,7],'v_max':self.v, 'colour':(225,225,0)}), #left lane, bottom to left

                (1, {'path': [1,17,9,13,10,22,7],'v_max':self.v, 'colour':(0,225,0)}), #left lane, right to left
                (1, {'path': [1, 17, 9,13,10,14,11,23,4],'v_max':self.v, 'colour':(0,225,0)}), #left lane, right to bottom

                (1, {'path': [2,18,10,14,11,23,4],'v_max':self.v, 'colour':(0,0,225)}), #left lane, top to bottom
                (1, {'path': [2,18,10,14,11,15,8,20,5],'v_max':self.v, 'colour':(0,0,225)}), #left lane, top to right
                
                (1, {'path': [3,19,11,15,8,20,5],'v_max':self.v, 'colour':(0,0,0)}), #left lane, left to right
                (1, {'path': [3,19,11,15,8,12,9,21,6],'v_max':self.v, 'colour':(0,0,0)}), #left lane, left to top

                (1, {'path': [24, 40, 32,44,29],'v_max':self.v, 'colour':(225,225,225)}), #bottom right turn
                (1, {'path': [25, 41, 33, 45, 30],'v_max':self.v, 'colour':(225,225,225)}), #right right turn
                (1, {'path': [26, 42, 34, 46, 31],'v_max':self.v, 'colour':(225,225,225)}), #top right turn
                (1, {'path': [27, 43, 35, 47, 28],'v_max':self.v, 'colour':(225,225,225)}), #left right turn

                (1, {'path': [24, 40, 32, 36, 33, 30],'v_max':self.v, 'colour':(225,0,225)}), #right lane, bottom to top
                (1, {'path': [25, 41, 33, 37, 34, 31],'v_max':self.v, 'colour':(225,0,225)}), #right lane, right to left
                (1, {'path': [26, 42, 34, 38, 35, 28],'v_max':self.v, 'colour':(225,0,225)}), #right lane, top to bottom
                (1, {'path': [27, 43, 35, 39, 32, 29],'v_max':self.v, 'colour':(225,0,225)}), #left right turn

                (1, {'path': [53, 48, 51,50,56],'v_max':self.v, 'colour':(225,0,0)}), #bottom ambulance lane
                (1, {'path': [54, 49, 52,50,56],'v_max':self.v, 'colour':(225,0,0)}), #left ambulance lane
                (1, {'path': [55, 56],'v_max':self.v, 'colour':(225,0,0)}), #top ambulance lane
                
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