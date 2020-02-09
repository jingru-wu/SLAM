import numpy as np
import math
import time
from matplotlib import pyplot as plt
from scipy.stats import norm
import pdb
from MapReader import MapReader

class SensorModel:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):
        """
        TODO : Initialize Sensor Model parameters here
        """
        self.occupancy_map=occupancy_map
        self.downsample=5

        ## initial parameter for learning model para
        ## z_hit,z_short,z_max,z_rand,var_hit,lambda_short
        self.params =[0.5, 0.2, 0.002,  0.398,   200.0,   0.003]
        # self.params = [0.3480,0.025,0.002,0.625,200.0,0.003]
        
        # self.laser_range = 8183 ## data log 1
        self.laser_range = 8191   ## data log 4


    def learn_params(self, measurements, true_ranges):
        """
        performance estimating parameters through EM method
        :param measurements:
        :param true_ranges:
        :return: updated parameters
        """
        z_hit,z_short,z_max,z_rand,var_hit,lambda_short= self.params
        pre_params=[z_hit,z_short,z_max,z_rand,var_hit,lambda_short]
        updated_params=[-1,-1,-1,-1,-1,-1]
        while np.max(np.abs(np.array(updated_params) - np.array(pre_params))) > 1e-6:

            e_hit, e_short, e_max, e_rand = [], [], [], []
            for i in range(len(measurements)):
                true_range, measurement = true_ranges[i], measurements[i]
                p_hit = self.PHit(true_range, measurement,var_hit)
                p_short = self.PShort(true_range, measurement,lambda_short)
                p_max = self.PMax(measurement)
                p_rand = self.PRand(measurement)
                normalizer = 1.0 / (p_hit + p_short + p_max + p_rand)
                e_hit.append(normalizer * p_hit)
                e_short.append(normalizer * p_short)
                e_max.append(normalizer * p_max)
                e_rand.append(normalizer * p_rand)
            e_hit, e_short, e_max, e_rand = np.array(e_hit), np.array(e_short), np.array(e_max), np.array(e_rand)

        # perform M step
            pre_params = [z_hit, z_short, z_max, z_rand, var_hit,lambda_short]
            z_hit = sum(e_hit) / len(measurements)
            z_short = sum(e_short) / len(measurements)
            z_max = sum(e_max)/ len(measurements)
            z_rand = sum(e_rand) / len(measurements)
            var_hit = np.sqrt(1.0 / np.sum(e_hit) * np.sum(e_hit * (np.array(measurements)-np.array(true_ranges))**2)).item()
            lambda_short = (np.sum(e_short) / np.sum(e_short * np.array(measurements))).item()
            updated_params = [z_hit, z_short, z_max, z_rand, var_hit, lambda_short]
        print('origin',self.params)
        print('updated',updated_params)
        return updated_params

    def PHit(self,z_measure_k, z_predict_k,var_hit):
        if z_measure_k < 0 or z_measure_k > self.laser_range:
            return 0
        normalizer = 1.0 / (norm(z_predict_k, var_hit).cdf(self.laser_range)
                            - norm(z_predict_k, var_hit).cdf(0))
        p = normalizer * norm(z_predict_k, var_hit).pdf(z_measure_k)
        return p

    def PMax(self,z_measure_k):
        if z_measure_k >= self.laser_range:
            return 1.0
        else:
            return 0

    def PShort(self,z_measure_k,z_predict_k,lambda_short):
        if z_measure_k < 0 or z_measure_k > z_predict_k:
            return 0
        normalizer = 1.0 / (1 - math.exp(-lambda_short * z_predict_k))
        p = normalizer * lambda_short * math.exp(-lambda_short * z_measure_k)
        return p

    def PRand(self,z_measure_k):
        if z_measure_k < 0 or z_measure_k >= self.laser_range:
            return 0
        else:
            return 1.0 / self.laser_range

    def ray_casting(self,x_t1):
        """
        :param x_t1: [x,y,theta] one state
        :return: a list of predicted distance in clockwise direction from laser location
        """
        map=self.occupancy_map
        offset=25

        [x,y,theta]=x_t1
        laser_x=x+offset*math.cos(theta)
        laser_y=y+offset*math.sin(theta)

        range_list=[]
        x_coords=[]
        y_coords=[]
        for the in range(-90,90,self.downsample):
            x_current=x+offset*math.cos(theta)
            y_current=y+offset*math.sin(theta)
            theta_current=theta+the/180*math.pi
            while(map[int(y_current/10),int(x_current/10)] and max(x_current,y_current)<8000 and min(x_current,y_current)>=0):
                x_current+=50*math.cos(theta_current)
                y_current+=50*math.sin(theta_current)
            range_list.append(math.sqrt((x_current-laser_x)**2+(y_current-laser_y)**2))
        # for display
            x_coords.append(int(x_current)/10)
            y_coords.append(int(y_current)/10)

        ### visualize ray casting during doing sensor model
        map_visual=0
        if map_visual:
            plt.imshow(map,'gray')
            a=plt.scatter(np.asarray(x_coords[:90]),np.asarray(y_coords[:90]),c='b',marker='.',linewidths=1)
            b=plt.scatter(np.asarray(x_coords[90:]),np.asarray(y_coords[90:]),c='g',marker='.',linewidths=1)
            c=plt.scatter(int(x/10),int(y/10),c='r',marker='o',linewidths=1)
            plt.show()
            plt.pause(0.01)
            a.remove()
            b.remove()
            c.remove()

        hist_visual=0
        if hist_visual:
            plt.hist(range_list,bins=200)
            # plt.show()
            plt.pause(0.01)
            # plt.close()
        return range_list

    def beam_range_finder_model(self, z_measure, x_t1):
        """
        param[in] z_measure : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] q : likelihood of a range scan z_measure_k1 at time t
        """
        z_predict=self.ray_casting(x_t1)
        q = 0
        z_measure=z_measure[0:-1:self.downsample]
        # [z_hit,z_short,z_max,z_rand,var_hit,lambda_short]=self.learn_params(z_measure,z_predict)
        # [z_hit,z_short,z_max,z_rand,var_hit,lambda_short]=[0.3480,0.025,0.002,0.625,200.0,0.003]
        [z_hit,z_short,z_max,z_rand,var_hit,lambda_short]=self.params
        for k in range(0,len(z_measure)):
            p = z_hit  * self.PHit(z_measure[k], z_predict[k],var_hit) + \
                z_short* self.PShort(z_measure[k], z_predict[k],lambda_short) + \
                z_max  * self.PMax(z_measure[k]) + \
                z_rand * self.PRand(z_measure[k])
            q += math.log(p)
        q=math.exp(q)
        if q==0:
            print('all 0')
            return 1e-200
        else:
            return q
