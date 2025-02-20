`Class TrafficSignal:`(`segments` : List[[Segment],[Segment]], `config`: { `cycle` =  [(False, True,60),(False,False,66), (True, False,126), (False, False, 132)], `slow_distance` = 40, `slow_factor` = 10, `stop_distance`} = 15, `current_cycle_index` = 0, `last_t` = 0)

`segments:` A list containing two lists (i.e a nested list), each nested list contains segments that will have traffic lights; The first nested segment will follow the first value in the nested tuples of `cycle`, while the second nested segment will follow the second value in the nested tuples of `cycle`.

`config:` A dictionary containing all the configurable properties of a TrafficSignal object, you can reassign these properties by defining them in config when initializing the TrafficSignal object.

`cycle:` A list of tuples; each tuple contains the states of the traffic lights and the time those states end. 

`slow_distance:` The distance before the stoplight where the vehicle starts to slow.

`slow_factor:` A factor defining the speed at which the vehicle slows down

`stop_distance:` The distance before the stoplight where a vehicle starts trying to stop.