import numpy as np

class Wheel(object):
    def __init__(self, distance_cg,drive=False):
        """

        """

        self.angular_speed = 0 #rad/s
        self.rubber_friction = 12.8 #pure
        self.radius = 0.34 #m
        self.mass = 35 # kg
        self.crr = 0.035 # pure
        # if the wheel is connected to the engine
        self.drive = drive
        # whatever the wheel is spinning
        self.spinning = False
        # longitudinal distance from the cg of the car
        self.distance_cg =  distance_cg
        self.c_friction = 1.0 #pure

    def get_slip_ratio(self, longitudinal_speed):
        """
        Get the current sleep ratio for the wheel
        :return:
        """
        slip_ratio = (self.angular_speed*self.radius-longitudinal_speed)/(longitudinal_speed + 1e-04)
        slip_ratio = np.clip(slip_ratio, -1, 1)
        return slip_ratio

    def get_inertia(self):
        return self.mass * self.radius**2 /2

    def get_rolling_force(self, speed):
        return speed * self.crr

    def get_max_traction(self, weight):
        """
        Get the maximum traction available before spinning
        :param weight:
        :return:
        """
        return self.c_friction * weight

    def get_weight_on_wheel(self, car):
        """
        get the weight on the wheel using weight transfer
        :param car_cg:
        :param car_acceleration:
        :param car_wheelbase:
        :return:
        """
        # todo this is for back traction
        # todo this also assume 4 wheels
        if self.drive:
            axel_weight = ((car.wheelbase - self.distance_cg) / car.wheelbase) * car.get_total_weight() + (
                        car.cg_height / car.wheelbase) * car.get_total_mass() * car.acceleration
            return axel_weight / 2
        else:

            return ((car.wheelbase - self.distance_cg)/car.wheelbase) * car.get_total_weight() - (car.cg_height/car.wheelbase)* car.get_total_mass() * car.acceleration


