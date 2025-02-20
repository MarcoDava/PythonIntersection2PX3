from base_intersection import *
import pandas as pd
import matplotlib.pyplot as plt
import time
import numpy as np
'''!!!IF YOUR SIMULATION IS RUNNING FOREVER CLICK ON THE COMMAND LINE THEN PRESS CMD+C !!!'''


start_time = time.time()
vehicle_times = []
time_variances = []
time_standard_deviations = []
data = []


#---------------------------------------------------------------Variables------------------------------------------------------#
simulation_time = 3600
number_of_sims = 5
#------------------------------------------------------------------------------------------------------------------------------#

def run_sim(sim, sim_time):
    sim.dt = 1/5
    sim.run(sim_time*5)
    vehicle_times.append(sim.get_average_vehicle_time())
    time_variances.append(sim.get_vehicle_time_variance())
    time_standard_deviations.append((sim.get_vehicle_time_variance()**(1/2)))

for i in range(number_of_sims):
    intersection = Intersection()
    temp_sim = intersection.get_sim()
    run_sim(temp_sim, simulation_time)
    data += temp_sim.vehicle_times

average_vehicle_time = round(np.average(vehicle_times),2)
average_time_variance = round(np.average(time_variances),2)
standard_deviation = round(np.average(time_standard_deviations),2)



print('\nAverage Travel Time: %s seconds' % (average_vehicle_time))
print('\nVehicle Time Variance: %s seconds' % average_time_variance)
print('\nVehicle Time Standard Deviation: %s seconds' % standard_deviation)
print("\n--- %s seconds ---\n" % (round(time.time() - start_time,2)))

#df = pd.DataFrame(data, columns = ['Travel_Time'])
#df.hist(bins=25)
#plt.xlabel('Travel Time(s)')
#plt.ylabel('Number of Vehicles')
#plt.show()




