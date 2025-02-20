`Class Segment`(`points` = [($x_0$, $y_0$), ($x_f$, $y_f$)], `config` = { `vehicles` = deque(), `width` = 3.5, `has_traffic_signal` = False})

`points:` A list containing the starting coordinates ($x_0$, $y_0$), and the ending coordinates ($x_f$, $y_f$) of the segment
<br>&ensp; &ensp; **When Implemented the Y coordinates get assigned the negative input of the value that was input**

`config:` A dictionary containing all the configurable properties of a segment object, you can reassign these properties by defining them in config when initializing the segment object.

`vehicles:` A deque containing all the vehicles that are driving on the segment in order.

`width:` the width of the segment.

`has_traffic_signal:` A boolean variable that is True when a segment contains a traffic signal and False when there is not one, this value automatically gets changes when you add a traffic signal.

**Segments is and Abstract Base Class (ABC); many of its methods are designed to be inhereted by sub-classes look  [here](https://docs.python.org/3/library/abc.html) for more details on how ABC's work**