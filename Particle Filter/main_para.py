import numpy as np
import sys
import pdb
from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling
import multiprocessing
import math
from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time



def binary_map(occupancy_map):
    """
    :param occupancy_map:
    :return: binarized map where 1 is inside map
    out1 corresponding to boundary    where 0.9< intensity <1
    out2 corresponding to inside area where 0=< intensity <0.1
    """
    out1=np.zeros(occupancy_map.shape)
    out2=np.zeros(occupancy_map.shape)
    out2[occupancy_map>0.9]=1
    out2[occupancy_map>1]=0
    out1[occupancy_map>=0]=1
    out1[occupancy_map>0.1]=0
    return out1+out2

def visualize_map(occupancy_map):
    fig = plt.figure()
    mng = plt.get_current_fig_manager()  # mng.resize(*mng.window.maxsize())
    plt.ion()
    plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800])

def visualize_timestep(X_bar, tstep,occupancy_map):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    displyray=1
    scat = plt.scatter(x_locs,y_locs, c='r', marker='.',linewidths=0.001)
    if displyray==1:
        [x_coords,y_coords]=visul_ray(X_bar,occupancy_map)
        a=plt.scatter(np.asarray(x_coords),np.asarray(y_coords),c='b',marker='.',linewidths=1)
    plt.title(tstep)
    plt.pause(0.001)
    scat.remove()
    if displyray==1:
        a.remove()


def visul_ray(X_bar,occupancy_map):
    """
    perform raycast again to oatain the coordinates
    :param X_bar:
    :param occupancy_map: binary map
    :return: x_coords,y_coords  coordinates of the end of ray, which is used for display
    """
    map=occupancy_map
    offset=25
    max_weight_index=np.argmax(X_bar[:,3])
    [x,y,theta]=X_bar[max_weight_index,:3]
    x_coords=[]
    y_coords=[]
    for the in range(-90,90,10):
        x_current=x+offset*math.cos(theta)
        y_current=y+offset*math.sin(theta)
        theta_current=theta+the/180*math.pi
        while(map[int(y_current/10),int(x_current/10)] and max(x_current,y_current)<8000 and min(x_current,y_current)>=0):
            x_current+=20*math.cos(theta_current)
            y_current+=20*math.sin(theta_current)
        x_coords.append(int(x_current)/10)
        y_coords.append(int(y_current)/10)
    return x_coords,y_coords

def init_particles_freespace(num_particles, occupancy_map):
    # initialize [x, y, theta] positions in world_frame for all particles
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) )
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )
    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles
    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    return X_bar_init

def init_particles_random(num_particles, occupancy_map):
    # initialize [x, y, theta] positions in occupancy_map for all particles
    y0_vals = np.random.randint( 0, 700, (num_particles*50, 1))
    x0_vals = np.random.randint( 300,700, (num_particles*50, 1))
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1))
    CoorInMap=[]
    count=0
    i=0
    while count<num_particles:
        if occupancy_map[y0_vals[i],x0_vals[i]]==1:
            CoorInMap.append([x0_vals[i][0],y0_vals[i][0]])
            count+=1
        i+=1

    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles
    X_bar_init = np.hstack((np.asarray(CoorInMap)*10,theta0_vals,w0_vals))
    return X_bar_init

def displayhist():
    """
    display hist for all laser measurement in datalog
    used for define parameter in sensor model
    """
    src_path_log = '../data/log/robotdata2.log'
    logfile = open(src_path_log, 'r')
    hist_list=[]

    for time_idx, line in enumerate(logfile):
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        if (meas_type == "L"):
            meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')
            ranges = meas_vals[6:-1]
            hist_list.extend(ranges)
    plt.hist(hist_list,bins=100)
    plt.show()

def combine_motion_sensor(x_t0,u_t0,u_t1,ranges,motion_model,sensor_model):
    """
    Combine two model for process one particle
    implemented in multiprocessing
    :param x_t0:   last guess states of one particle
    :param u_t0:   last robot states in odometry
    :param u_t1:   current robot states in odometry
    :param ranges:  z measurement
    :param motion_model:
    :param sensor_model:
    :return: updated Xbar[m,:] =[x_t1,w_t]1*4  where x_t1:1*3 w_t:1*1
    """
    x_t0=x_t0[:3]
    x_t1 = motion_model.update(u_t0, u_t1, x_t0)
    w_t = sensor_model.beam_range_finder_model(ranges, x_t1)
    return np.hstack([x_t1,w_t])

def main():
    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """
    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata2.log'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()

    vis_flag = 1
    if vis_flag:
        visualize_map(occupancy_map)

    logfile = open(src_path_log, 'r')
    occupancy_map=binary_map(occupancy_map)
    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 500
    X_bar = init_particles_random(num_particles, occupancy_map)

    print('X_bar shape:',X_bar.shape)

    ##    Monte Carlo Localization Algorithm : Main Loop
    first_time_idx = True
    for time_idx, line in enumerate(logfile):
        start_time=time.time()
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        if (meas_type == "L"):
            meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ')
            odometry_robot = meas_vals[0:3]
            time_stamp = meas_vals[-1]
            odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
            ranges = meas_vals[6:-1]
            if (first_time_idx):
                u_t0 = odometry_robot
                first_time_idx = False
                continue
            u_t1 = odometry_robot

            if sum(abs(u_t1[:2]-u_t0[:2]))<1 and abs(u_t1[2]-u_t0[2])<(3.14/20):
                print('skip') ## skip states that doesn't moved
                continue
            if vis_flag:
                visualize_timestep(X_bar, time_idx,occupancy_map)
            # impplement multiprocessing for
            args=[[X_bar[m],u_t0,u_t1,ranges,motion_model,sensor_model] for m in range(0, num_particles)]
            with multiprocessing.Pool(processes=16) as p:
                res=p.starmap(combine_motion_sensor, args)
            X_bar = np.asarray(res)
            # np.save('log2_Xbar/'+str(time_idx)+'.npy',X_bar) # save Xbar for accelerate display
            u_t0 = u_t1

        ## RESAMPLING
            X_bar = resampler.low_variance_sampler(X_bar)

            end_time=time.time()
            print("Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s")
            print('time_cost for 1 time step:',end_time-start_time)
            if time_idx>300: # stop display when time_idx>300, for saving time
                vis_flag=0

if __name__=="__main__":
    main()
    # displayhist()

