`class Simulation`:()

# Variables
`segments = [] :` A list containing all segments added to the simulation.

`vehicles = {} :` A dictionary storing all vehicles as values under their uuid as a key.

`vehicle_generator = [] :` A list containing all vehicle generators added to the simulation.

`traffic_signals = [] :` A list containing all traffic signals added to the simulation.

`t = 0.0 :` The simulation time, it increases by `dt` every time the simulation updates.

`frame_count = 0 :` The number of frames that have been run, increases by 1 every time the simulation updates.

`dt = 1/60 :` the amount the simulation time increases every update

`interfearing_paths = [] :` A list containing all lists of segements that interfear with each other.

`vehicle_times = [] :` A list storing the travel time of every vehicle that has been through the intersection.

# Methods
`define_interfearing_paths(interfearing, other,turn):` Defines interfearing paths

interfearing : [entrance segment reference number to interfearing segment, interfearing segment reference number  (usually a turn)]

other: [entrance segment reference number to segment that is interfeared with, segment reference number that is interfeared with]

turn: a boolean value that is true if the interfearing segment is a turn

`add_vehicle(Vehicle : veh):` Adds a vehicle to the simulation

`add_segment(Segment: seg):` Adds a segment to the simulation

`add_vehicle_generator(VehcileGenerator : gen):` Adds a vehicle generator to the simulation

`create_vehicle(**kwargs):` Creates a vehicle based on an undefined amount of keyword inputs(thats what **kwargs is for). You must assign the vehicles path and all other config inputs are optional then assigning it to a dictionary containing the key value pairs you would like to change. For example create_vehicle(config  = {'path': [0, 16, 12], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance})

`create_segment(*args):` Attempts to create a segment base on an undefined amount of inputs (*args). You must define the points of the segment [($x_0$, $y_0$), ($x_f$, $y_f$)] and optionally you can also define the config = { `vehicles` = deque(), `width` = 3.5, `has_traffic_signal` = False}

`create_quadratic_bezier_curve(start, control, end):` Creates a quadratic curve segment based on the start control and end inputs; (`start` = ($x_0$, $y_0$), `control` = ($x_c$, $y_c$), `end` = ($x_f$, $y_f$))

`create_cubic_bezier_curve(start, control_1, control_2, end):` Creates a cubic curve segment based on the start control_1, control_2, and end inputs; (`start` = ($x_0$, $y_0$), `control_1` = ($xc_1$, $yc_1$), `control_2` = ($xc_2$, $yc_2$), `end` = ($x_f$, $y_f$))

`create_vehicle_generator(**kwargs):` Creates a Vehicle Generator based on a undefined amount of keywork inputs. You must define the config of the vehicle generator and inside the config the vehicle must be defined, all other inputs are optional. For Example:
config = {'vehicles': [(1, {'path': [0, 16, 12], 'v_max': 20})], vehicle_rate' = 20 }

`create_traffic_signal(*args):` Creates a traffic signal based on a undifined amount of inputs. You must define the segments that the traffic signal is on. For example: simulation.create_traffic_signal([Segment 1, Segment 2],[Segment 3, Segment 4])

`add_traffic_signal(sig):` Adds Traffic signal sig

`run(steps):` runs update() steps number of times

`get_average_vehicle_time():` returns the average vehicle time

`get_vehicle_time_variance():` returns the average vehicle

`update():` updates a single frame