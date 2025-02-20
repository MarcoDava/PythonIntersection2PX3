`class Vehicle`(<br>
&ensp; config = {<br>
&ensp; &ensp; id = uuid.uuid4(), w = 1.74, l = 4, s0 = 8, T = 1, v_max = 16.6, a_max = 5, b_max = 4.61, time = 0, path=[], current_road_index = 0, x = 0 , v = 16.6 , a = 0<br>
&ensp; }<br>
)<br>
`config`: A dictionary containing all the configurable properties of a vehicle object, you can reassign these properties by defining them in config when initializing a vehicle object.

`id`: A uuid4 object that is used as a unique identifier for the vehicle. It is recommended not to change it from it's default value.

`w`: The width of the vehicle 

`l`: The length of the vehicle

`s0`: the distance the vehicle will try too keep from vehicles ahead of it in the same segment

`T` : the reaction time of the vehicle

`v_max` : the maximum desired speed of the vehicle

`a_max` : the maximum acceleration of the vehicle

`b_max` : the maximum decceleration of the vehicle

`time` : the time the vehicle has existed

`path` : an ordered list containing the indexes of segments the vehicle will visit. **It is reccomended to define the path when initializing a vehicle.**

`current_road_index` : an integer that reference the index of the current segment in path the vehicle is in.

`x` : the position along the current segment the vehicle is.

`v` : the current speed of the vehicle.

`a` : the current acceleration of the vehicle.

`set_default_config()` : assigns every config variable its default value(s)

`init_properties` : initializes two values `sqrt_ab` and `_v_max`.

`sqrt_ab` : contains the result of this equation $2\sqrt{a\_max*b\_max}$

`_v_max` : holds the `v_max` value that the vehicle was initialized with.

`update(lead, dt)`: <br>Uses an second order taylor series estimate of [The Intelligent Driver Model](https://journals.aps.org/pre/abstract/10.1103/PhysRevE.62.1805), developed by Treiber, Hennecke et Helbing, to update the current position `x`, speed `v`, acceleration `a`, and `time` of the vehicle. This [article](https://towardsdatascience.com/simulating-traffic-flow-in-python-ee1eab4dd20f) further details how it is implemented in this simulation software.