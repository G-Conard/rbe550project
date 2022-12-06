from Vector2D import Vector2D

class APF:
    def __init__(self, F_att, K_rep, OBS_DIST_MAX):
        self.__F_att=F_att # ATTRACTIVE FORCE MAGNITUDE
        self.__K_rep = K_rep # REPULSIVE FORCE CONSTANT
        self.__OBS_DIST_MAX=OBS_DIST_MAX # MAX OBSTACLE DISTANCE

    def __attractive_force(self, psi):
        return Vector2D(magnitude=self.__F_att,alpha=psi)

    def __repulsive_force(self, obstacles):
        rep = Vector2D(0, 0)
        for obstacle in obstacles:
            if (obstacle.getMagnitude() <= self.__OBS_DIST_MAX):
                obsMag=obstacle.getMagnitude()
                obsDir=-obstacle.getDirection()
                rep += Vector2D(obsDir[0], obsDir[1]) * self.__K_rep * (1.0 / obsMag - 1.0 / self.__OBS_DIST_MAX) / (obsMag ** 2)
        return rep

    def plan(self, psi, obstacles):
        f_total = self.__attractive_force(psi) + self.__repulsive_force(obstacles)
        return f_total.getDirection()