import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import time as time
import datetime
import pytz
import math
import matplotlib as mpl
import matplotlib.cm as cm
import csv
import dweepy


COMMAND_IOT_NAME = 'turtlebot_patrol_command'
STATUS_IOT_NAME = 'turtlebot_patrol_status'

CMD_GOTO = "goto"
CMD_STANDBY = "standby"

ROS_ERROR = "[ROS ERROR]"
SUCCESS = "success"


robot_sensor = 'CrestResearchData-6' # robot sensor

wait_time = 7 # time to stay at each point

X, Y = 5.5, 5.5 # size of the space

delta_x, delta_y = 0.7, 0.7 # robot step size

x_grid, y_grid = None, None

trans_x, trans_y = -3.14308, -3.01981 # transform from my coordinate to ROS coordinate

var = 1.0 # variance of the gaussian weight function

cutoff_radius = 6 # cutoff radius for the update_map function


initial_pose = [1, 1]

start_time = 0

def ROS_point(actual_x, actual_y):
	return actual_x * delta_x + trans_x, actual_y * delta_y + trans_y


# Send a goal to the robot and monitor its movement
# Return True if robot successfully arrives, False otherwise 
def goto(x, y):
	x, y = ROS_point(x, y)
	str_point = str(x) + ' ' + str(y)
	dweepy.dweet_for(COMMAND_IOT_NAME, {CMD_GOTO: str(x) + ' ' + str(y)})
	while True:
		status = dweepy.get_latest_dweet_for(STATUS_IOT_NAME)[0]['content']['status']
		if str_point in status:
			break
		time.sleep(1)

	standby()
	
	if(SUCCESS in status):
		return True
	else:
		return False


def goto_vicinity(x, y):
	mdist = 1
	while True:
		for abs_x in range(mdist+1):
			abs_y = mdist-abs_x
			
			if goto(x + abs_x, y + abs_y) == True:
				return x + abs_x, y + abs_y
			
			if goto(x + abs_x, y - abs_y) == True:
				return x + abs_x, y - abs_y
			
			if goto(x - abs_x, y + abs_y) == True:
				return x - abs_x, y + abs_y
			
			if goto(x - abs_x, y - abs_y) == True:
				return x - abs_x, y - abs_y

		mdist += 1




def standby():
	dweepy.dweet_for(COMMAND_IOT_NAME, {CMD_STANDBY: ''})
	time.sleep(1)
	dweepy.dweet_for(STATUS_IOT_NAME, {'status':''})
	time.sleep(1)



def visualize(robot_map, route=None, colorbar=True):
	radius = 2.0
	source_index = np.argmax(robot_map)
	source_x = source_index // len(y_grid)
	source_y = source_index % len(y_grid)
	print('Predicted source: ' + str(source_x) + ', ' + str(source_y))

	fig = plt.figure()

	robot_map = np.swapaxes(robot_map,0,1)
	cmap = plt.get_cmap('RdBu_r')
	norm = mpl.colors.PowerNorm(gamma=0.99, vmin=np.min(robot_map),vmax=np.max(robot_map))
	m = cm.ScalarMappable(norm=norm, cmap=cmap)

	if(route!=None):
		route = np.asarray(route)
		plt.plot(radius * route[:,0], radius * route[:,1], '-ko')

	plt.imshow(np.kron(robot_map, np.ones((radius,radius))), cmap=cmap, norm=norm,aspect='auto')
	plt.gca().invert_xaxis()

	if(colorbar):
		plt.colorbar()

	start_time_str = str(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(start_time)))
	sec_elapsed = int(time.time() - start_time)
	fig_name = 'figs/' + start_time_str + ' ' + str(sec_elapsed) + '.png'

	fig.savefig(fig_name, bbox_inches='tight')


def gweight(measure_pt, interest_pt):
    measure_pt, interest_pt = np.asarray(measure_pt), np.asarray(interest_pt)
    square_dist = np.linalg.norm(measure_pt - interest_pt)
    return np.exp(-square_dist/(2*var)) / math.sqrt(var)

def update_map(robot_map, weight_map, x, y, ppm):
    for j in range(-cutoff_radius, cutoff_radius):
        for i in range(-cutoff_radius, cutoff_radius):
            X, Y = x+i, y+j
            if(X not in range(len(x_grid)) or Y not in range(len(y_grid))):
                pass
            else:
                w = gweight([x,y], [X,Y])
                robot_map[X,Y] = (1.0-w)*robot_map[X,Y] + w * ppm
                weight_map[X,Y] += w

def initialize():
	standby()

	os.system("rm -r figs")
	os.system("mkdir figs")

	global x_grid
	global y_grid


	x_grid, y_grid = np.arange(0, X, delta_x),np.arange(0, Y, delta_y)

	print 'Initializing...'

	initial_ppm = observe()

	robot_map = initial_ppm * np.ones((len(x_grid), len(y_grid)))
	weight_map = np.zeros((len(x_grid), len(y_grid)))

	global start_time
	start_time = time.time()

	return robot_map, weight_map


def neighbors(x, y):
    candidates = [[x+1,y],[x-1,y],[x,y+1],[x,y-1]]
    neighbor_poses = []
    for n in candidates:
        if(n[0] in range(len(x_grid)) and n[1] in range(len(y_grid))):
            neighbor_poses.append(n)
    return neighbor_poses


def stay_penalty():
	min_elapsed = (time.time() - start_time) / 60
	return max(0, 1000 - 100*min_elapsed)


def observe():

	t = time.time()
	
	print 'Observe at time: ' + str(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)))

	time.sleep(wait_time)

	# read = np.nan

	# while(read != np.nan):

	message = dweepy.get_latest_dweet_for(robot_sensor)

	read = message[0]['content']['CO2']

	time = message[0]['created']

	# time.sleep(5)

	
	print 'Reading: ' + read
	print 'Measure time: ' + time

	# data = pd.read_csv("Station_Data.csv",sep = " ", skiprows=1200000, parse_dates = [[0, 1]])
	# data.columns = ['Time', 'Station_num', 'UV_index', 'PM01', 'CO2', 'Temperature', 'PM10', 'IR', 'Visibility', 'Humidity', 'Organic_vapor', 'PM25']
	# data['time_stamp'] = data['Time'].apply(lambda x:datetime.datetime.strptime(x[1:20],"%Y-%m-%d %H:%M:%S"))

	
	# data = data[data.time_stamp > time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t))]
	# data = data[data.Station_num == robot_sensor]
	
	# print str(data.shape[0]) + ' measurements'
	# print 'From ' + str(data['time_stamp'].iloc[0]) + ' to ' + str(data['time_stamp'].iloc[data.shape[0] - 1])
	# print 'Average reading: ' + str(np.nanmean(data['CO2']))

	return read


def gradient_method(bound=3, penalty=True):
	
	robot_map, weight_map = initialize()

	goto(initial_pose[0], initial_pose[1])

	curr_pose = initial_pose
	route = [initial_pose]

	while True:

		print 'At location ' + str(curr_pose)
		
		ppm = observe()
		update_map(robot_map, weight_map, curr_pose[0], curr_pose[1], ppm)

		visualize(robot_map, route)

		base_pose = list(curr_pose)

		neighbor_poses = neighbors(curr_pose[0], curr_pose[1])
		for neighbor_pose in neighbor_poses:
			if neighbor_pose in route:
				pass
			else:
				if(goto(neighbor_pose[0], neighbor_pose[1]) == True):
					print 'Arrived at neighbor ' + str(neighbor_pose)
					route.append(neighbor_pose)
					curr_pose = neighbor_pose
					ppm = observe()
					update_map(robot_map, weight_map, curr_pose[0], curr_pose[1], ppm)
				else:
					print 'Failure going to neighbor ' + str(neighbor_pose)

		decision_map = robot_map.copy()
		if(penalty):
			decision_map[base_pose[0], base_pose[1]] -= stay_penalty()


		if(bound == None):
			next_index = np.argmax(decision_map)
			next_pose = [next_index // len(y_grid), next_index % len(y_grid)]

		else:
			curr_max = 0
			next_x, next_y = base_pose[0], base_pose[1]
			for j in xrange(-bound, bound+1, 1):
				for i in xrange(-bound, bound+1, 1):
					x, y = base_pose[0] + i, base_pose[1] + j
					if x in range(len(x_grid)) and y in range(len(y_grid)):
						p = decision_map[x, y]
						if p >= curr_max:
							curr_max = p
							next_x, next_y = x, y
			next_pose = [next_x, next_y]


		if(next_pose == base_pose):
			print 'Terminated at: ' + str(base_pose)
			print 'Total time elapsed: ' + str(int((time.time() - start_time) / 60))
			break

		else:
			if(goto(next_pose[0], next_pose[1]) == True):
				route.append(next_pose)
				curr_pose = next_pose
			else:
				print "Failure going to next position " + str(next_pose)
				print "Try to go to a nearby point..."
				next_x, next_y = goto_vicinity(next_pose[0], next_pose[1])
				route.append([next_x, next_y])
				curr_pose = [next_x, next_y]
				print "Arrived at vicinity " + str(curr_pose)


if __name__ == "__main__":
	# gradient_method(bound=3, penalty=True)

	while True:
		observe()







	

	