from scenario.Scenario import load_scenario, create_dev_file, Position, \
    LineSegment
import pylab as plt
import numpy as np


def closest_point(point, point_arr):
    mag = lambda p: np.sqrt(p[0] ** 2 + p[1] ** 2)
    point_mag = mag(point)
    closest = None
    _closest_point = None
    for target_point in point_arr:
        mag_diff = mag(target_point) - point_mag
        if closest is None or mag_diff < closest:
            closest = mag_diff
            _closest_point = target_point
    return _closest_point


class Simulation(object):

    def __init__(self, scenario_path, render=False, draw_env=False):
        self.scenario = load_scenario(scenario_path)
        if render:  # Draww environment start
            self.f, self.ax = plt.subplots()
            self.ax.plot(0, 0, 'bo')  # Origin
            if draw_env:
                self._draw_rectangle(self.scenario.room)
                self._draw_rectangle(self.scenario.target)
        self.t = 0
        self.last_sample = 0
        self.stepped_step = False
        self.lidar_pos = Position(self.scenario.sensor.radius, 0)
        self.environment = (self.scenario.room, self.scenario.target)
        self.sampling = False
        self.rendering = render
        self.write_output()
        #self.step(0)

    def step(self, t):
        self.stepped_step = True
        self.t = t
        self.rotate_lidar()

    def step_time(self):
        if self.stepped_step: raise PermissionError(
            "Step time called after step, WRONG API USAGE!")
        self.t += self.scenario.step_size_s
        self.move_target()
        self.control_lidar()
        self.sample_lidar()

    def control_lidar(self):
        # todo: V profile should be function of t and output velocity in mm/s
        # V profile is how fast it turns in one sample period
        d_theta = self.scenario.sensor.v_profile * \
                  self.scenario.step_size_s
        self.lidar_pos.pos[1] += d_theta

    def sample_lidar(self):
        self.last_sample += self.scenario.step_size_s
        if self.last_sample >= self.scenario.sensor_sample_rate_s:
            self.last_sample = 0
            self.sampling = True
            laser = self._laser()
            intersection = []
            for obj in self.environment:
                intersection.extend(obj.get_laser_points(laser))
            intersection = closest_point(self.lidar_pos.to_cartesian(),
                                         intersection)

            if self.rendering:
                self.ax.plot(*intersection, 'ro')
        else:
            self.sampling = False

    def move_target(self):
        # todo: target_v should be function of t and output velocity in mm/s
        dx = self.scenario.step_size_s * self.scenario.target_v[0]
        dy = self.scenario.step_size_s * self.scenario.target_v[1]
        self.scenario.target.x += dx
        self.scenario.target.y += dy

    def rotate_lidar(self):
        self.lidar_pos.set_theta(self.scenario.sensor.v_profile[0] * self.t)
        laser = self._laser()
        intersection = []
        for obj in self.environment:
            intersection.extend(obj.get_laser_points(laser))
        intersection = closest_point(self.lidar_pos.to_cartesian(),
                                     intersection)

        if self.rendering:
            self.ax.plot(*intersection, 'ro')

    def write_output(self):
        pass

    def render(self, draw_laser=False):
        if not  self.rendering: return
        lidar_pos = self.lidar_pos.to_cartesian()
        self.ax.plot(lidar_pos[0], lidar_pos[1], 'ro')
        theta = self.lidar_pos.pos[1]
        xf = self.scenario.sensor.sensor_range[1] * np.cos(theta)
        yf = self.scenario.sensor.sensor_range[1] * np.sin(theta)
        #self._draw_rectangle(self.scenario.target)
        if draw_laser and self.sampling:
            laser = LineSegment(
                lidar_pos[0] + self.scenario.sensor.sensor_range[0] * np.cos(theta),
                lidar_pos[1] + self.scenario.sensor.sensor_range[0] * np.sin(theta),
                xf, yf)
            laser_points = laser(2)
            self.ax.plot(laser_points[0], laser_points[1], 'r-')

    def _draw_rectangle(self, rectangle):
        top, left, bottom, right = rectangle.get_equations()
        top_x, top_y = top(2)
        self.ax.plot(top_x, top_y, 'b-')
        left_x, left_y = left(2)
        self.ax.plot(left_x, left_y, 'b-')
        bottom_x, bottom_y = bottom(2)
        self.ax.plot(bottom_x, bottom_y, 'b-')
        right_x, right_y = right(2)
        self.ax.plot(right_x, right_y, 'b-')

    def _laser(self):
        lidar_pos = self.lidar_pos.to_cartesian()
        theta = self.lidar_pos.pos[1]
        xf = self.scenario.sensor.sensor_range[1] * np.cos(theta)
        yf = self.scenario.sensor.sensor_range[1] * np.sin(theta)
        return LineSegment(
                lidar_pos[0] + self.scenario.sensor.sensor_range[0] * np.cos(theta),
                lidar_pos[1] + self.scenario.sensor.sensor_range[0] * np.sin(theta),
                xf, yf)


if __name__ == "__main__":
    # todo Make everything relative to updae rate of sensor
    dev_scenario = "scenarios/dev.yaml"
    create_dev_file(dev_scenario)
    scenario_path = dev_scenario
    sim = Simulation(scenario_path, render=True, draw_env=True)
    #sim.render()
    for _ in range(200):
        sim.step_time()
        sim.render(draw_laser=False)
    # x = 300
    # for t in range(360):
    #     sim.step(t)
    #     sim.render(draw_laser=False)
    plt.show()
    #scenario.step(1)
