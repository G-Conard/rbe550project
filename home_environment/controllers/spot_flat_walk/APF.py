from Vector2D import Vector2D
import numpy as np


class APF:
    def __init__(self, F_att, K_rep, OBS_DIST_MAX):
        self.__F_att=F_att # ATTRACTIVE FORCE MAGNITUDE
        self.__K_rep = K_rep # REPULSIVE FORCE CONSTANT
        self.__OBS_DIST_MAX=OBS_DIST_MAX # MAX OBSTACLE DISTANCE

    def __attractive_force(self, psi):
        # Define vector with attractive force magnitude and direction
        return Vector2D(magnitude=self.__F_att,alpha=psi)

    def __repulsive_force(self, obstacles):
        # Compute repulsive force based on LIDAR readings
        rep = Vector2D(0, 0)
        for obstacle in obstacles:
            if (obstacle.getMagnitude() <= self.__OBS_DIST_MAX):
                obsMag=obstacle.getMagnitude()
                obsDir=-obstacle.getDirection()
                rep += Vector2D(obsDir[0], obsDir[1]) * self.__K_rep * (1.0 / obsMag - 1.0 / self.__OBS_DIST_MAX) / (obsMag ** 2)
        return rep

    def plan(self, psi, obstacles):
        # Calculate direction from sum of attractive and repulsive forces
        f_total = self.__attractive_force(psi) + self.__repulsive_force(obstacles)
        return f_total.getDirection()

    def cart2pol(self, x, y):
        # Convert from Cartesian coordinates (x and y) to polar coordinates (magnitude and direction)
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return Vector2D(rho, phi)

    def pol2cart(self, rho, phi):
        # Convert from polar coordinates (magnitude and direction) to Cartesian coordinates (x and y)
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return Vector2D(x, y)
        
    def fview(self,theta0,thetaf):
        # Define array of angles in radians corresponding with number of points in one lidar layer
        # left (positive) to right (negative)
        angle = np.linspace(theta0*np.pi/180,-thetaf*np.pi/180,512)
        return angle

    def obstacles(self,image,angle,coor): 
        # Make list of LIDAR readings
        # Image: 1 front, 2 rear, 3 floor
        # Choose coordinate system: coor = 1 polar, coor=2 cartesian
        list=[]
        if coor ==1:
            for i in range(0,512):
                list.append(Vector2D(image[i], angle[i]))
        if coor ==2:
            for i in range(0,512):
                list.append(self.pol2cart(image[i], angle[i]))
        #print('Front obstacles:','{:.2f}' .format(front[0:8])
        return list