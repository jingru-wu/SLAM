import numpy as np
import pdb
import random
import matplotlib.pyplot as plt
class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """
    def low_variance_sampler(self, X_bar):
        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        weights=X_bar[:, 3]
        ## normalize weight:
        weights=weights/sum(weights)
        # print(len(weights))
        M=X_bar.shape[0]

        X_bar_resampled=[]
        r = np.random.uniform(0, 1./M)
        c = weights[0]
        i = 0
        for m in range(1,M+1):
            u = r + (m-1)/M
            while u > c:
                i += 1
                c += weights[i]
            X_bar_resampled.append(X_bar[i])
        X_bar_resampled=np.array(X_bar_resampled)
        # X_bar_resampled[:,-1]=X_bar_resampled[:,-1]/np.sum(X_bar_resampled[:,-1])  ## implement gaussian sample around [x,y]
        return X_bar_resampled

def gaussian_sample(X):
    X[0]=np.round(np.random.normal(X[0],10,1))
    X[1]=np.round(np.random.normal(X[1],10,1))
    X[2]=X[2]+np.random.uniform(-0.3,0.3)*np.pi
    return X

if __name__ == "__main__":
    pass
