`Class VehicleGenerator` ( `config` = { `vehicle_rate` = 10, `vehicles`=[ 1,`vehicle.config` = { id = uuid.uuid4(), w = 1.74, l = 4, s0 = 8, T = 1, v_max = 16.6, a_max = 5, b_max = 4.61, time = 0, path=[], current_road_index = 0, x = 0 , v = 16.6 , a = 0} ] } )

`config:` A dictionary containing all the configurable properties of a VehicleGenerator object, you can reassign these properties by defining them in config when initializing the VehicleGenerator object.

`vehicle_rate:` the aveerage time at which the vehicles spawn is defined by this equation: $50/(vehicle\_rate)$ seconds.

`vehicles:` A list containing tuples, the second value in the tuple is a config for a vehicle, and the first value is the weight of how often the vehicle defined by the config spawns.

For Example: if you define 2 vehicles each with a weight of 1, each vehicle has a 1 in 2 chance of spawning, then if you add a vehicle with a weight of 2, that vehicle has a 2 in 4 chance of spawning while the first two vehicles now have a 1 in 4 chance of spawning.