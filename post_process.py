import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ScenarioOutput(object):

    def __init__(self, data_arr):
        self.time = data_arr[:, 0]  # s
        self.target_r = np.sqrt(data_arr[:, 1] ** 2 +
                                data_arr[:, 2] ** 2)  # mm
        self.target_theta = np.arctan(data_arr[:, 2] / data_arr[:, 1])  # rad
        self.target_center_r = np.sqrt(data_arr[:, 3] ** 2 +
                                       data_arr[:, 4] ** 2)  # mm
        self.target_center_theta = np.arctan(data_arr[:, 4] /
                                             data_arr[:, 3])  # rad
        self.target_v_x = data_arr[:, 5]  # mm/s
        self.target_v_y = data_arr[:, 6]  # mm/s
        self.sensor_v = data_arr[:, 7]  # rad/s


def scenario_output_to_df(output_path):
    all_data = []
    for i in range(len(os.listdir(output_path))):
        output_file = f"{scenario_name}{i}.csv"
        full_output_path = os.path.join(output_path, output_file)
        data = np.loadtxt(full_output_path,
                          delimiter=",", skiprows=1, dtype=object)
        is_target = data[:, 1]
        data = np.hstack((data[:, 0].reshape((-1, 1)), data[:, 2:])).astype(float)
        points_of_interest = np.where(is_target == 'True')[0]
        data = data[points_of_interest, :]
        all_data.append(ScenarioOutput(data))
    return all_data




if __name__ == "__main__":
    scenario_name = "dev"
    output_path = os.path.join("scenarios", "output", scenario_name)
    data = scenario_output_to_df(output_path)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for d in data:
        ax.plot(d.target_r, d.target_theta)

    plt.show()
