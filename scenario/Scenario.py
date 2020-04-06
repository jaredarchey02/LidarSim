import os
import yaml
from dataclasses import dataclass
from typing import List
import numpy as np

"""

3D plot showing sensor angular velocity, Object velocity,
FOM



"""

def cart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan(y / x)

def create_dev_file(outfile, vbias):
    sensor = LidarSensor(10,             # Sensor radius
                         [100, 10000],   # Sensor range
                         2 * np.pi * 5)  # Sensor velocity rad/s
    room = Rectangle(0,      # Room x
                     0,      # Room y
                     10000,  # Room width
                     10000,  # Room height
                     0)      # Room rotation
    target = Rectangle(2000,   # Target x
                       -12000,  # Target y
                       20000,   # Target width
                       20000,     # Target height
                       0)      # Target rotation
    step_size_s = 1e-3
    sensor_sample_rate_s = step_size_s
    target_v = [
        -2000 + vbias,  # x velocity mm/s
        0]      # y velocity mm/s
    scenario = Scenario(step_size_s, sensor_sample_rate_s, sensor,
                        room, target, target_v)
    with open(outfile, 'w') as f:
        yaml.dump(scenario, f)


def load_scenario(fname):
    with open(fname, 'r') as f:
        scenario = yaml.load(f, Loader=yaml.FullLoader)
    return scenario


def between(p, x, tol=1e-6):
    x_min = np.min(p)
    x_max = np.max(p)
    #print(p, x)
    #print(x_min, x, x_max, p)
    return (x_min - tol) <= x <= (x_max + tol)


class LineSegment(object):

    @staticmethod
    def _line_coeffs(p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0] * p2[1] - p2[0] * p1[1])
        return A, B, -C

    def __init__(self, xi, yi, xf, yf):
        self.x = (xi, xf)
        self.y = (yi, yf)

    def __call__(self, n):
        return np.linspace(self.x[0], self.x[1], n), \
               np.linspace(self.y[0], self.y[1], n)

    def intersection(self, other):
        # https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
        x1i, x1f = np.linspace(self.x[0], self.x[1], 2)
        y1i, y1f = np.linspace(self.y[0], self.y[1], 2)
        x2i, x2f = np.linspace(other.x[0], other.x[1], 2)
        y2i, y2f = np.linspace(other.y[0], other.y[1], 2)
        A1, B1, C1 = LineSegment._line_coeffs((x1i, y1i), (x1f, y1f))
        A2, B2, C2 = LineSegment._line_coeffs((x2i, y2i), (x2f, y2f))
        D = A1 * B2 - B1 * A2
        Dx = C1 * B2 - B1 * C2
        Dy = A1 * C2 - C1 * A2
        if D != 0:
            x = Dx / D
            y = Dy / D
            if not between(self.x, x) or not between(other.x, x):
                x = np.nan
            elif not between(self.y, y) or not between(other.y, y):
                y = np.nan

            return x, y
        else:
            return False


class Position(object):

    def __init__(self, r, theta):
        self.pos = np.array([r, theta], dtype=np.float64)

    def set_theta(self, theta):
        self.pos[1] = theta

    def to_cartesian(self):
        return np.array([self.pos[0] * np.cos(self.pos[1]),
                         self.pos[0] * np.sin(self.pos[1])])


@dataclass
class Rectangle(object):

    x: float  # Top left corner x, y
    y: float
    width: float
    height: float
    theta: float  # Radians

    def get_equations(self):

        center_x = self.x
        center_y = self.y
        center = np.array([center_x, center_y])

        # center_x = self.x + (self.width / 2)
        # center_y = self.y - (self.height / 2)
        #print(self.width, self.height)
        r = np.sqrt((self.width / 2) ** 2 + (self.height / 2) ** 2)
        top_left = Position(r, 3 * np.pi / 4 + self.theta).to_cartesian() \
            + center
        top_right = Position(r, np.pi / 4 + self.theta).to_cartesian() \
            + center
        bottom_left = Position(r, 5 * np.pi / 4 + self.theta).to_cartesian() \
            + center
        bottom_right = Position(r, 7 * np.pi / 4 + self.theta).to_cartesian() \
            + center

        top = LineSegment(*top_left, *top_right)
        left = LineSegment(*top_left, *bottom_left)
        bottom = LineSegment(*bottom_left, *bottom_right)
        right = LineSegment(*bottom_right, *top_right)

        return (top, left, bottom, right)


    def get_laser_points(self, laser):
        sides = self.get_equations()
        intersection = []
        for side in sides:
            intersect = laser.intersection(side)
            if not True in np.isnan(intersect):
                intersection.append(intersect)
        return intersection


@dataclass
class LidarSensor(object):
    """


    Class Attributes
    ----------------
    radius: int
        radius of the sensor in units of mm. The sensor traverses this
        radius
    sensor_range: List[int]
        Min and max distance of the sensor in mm
    v_profile: List[float]
        Velocity at timesteps in units of rad/t
    """
    radius: float
    sensor_range: List[float]
    v_profile: float


@dataclass
class Scenario(object):
    step_size_s: float
    sensor_sample_rate_s: float
    sensor: LidarSensor
    room: Rectangle
    target: Rectangle
    target_v: List[float]


if __name__ == "__main__":
    create_dev_file()
    fname = "../scenarios/dev.yaml"
    load_scenario(fname)