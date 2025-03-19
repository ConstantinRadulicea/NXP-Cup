import matplotlib.pyplot as plt
import numpy as np
import time


def line_to_points(line, x_range=(-10, 10)):
    A, B, C = line
    if B == 0:
        x1, x2 = -C / A, -C / A
        y1, y2 = x_range
    else:
        x1, x2 = x_range
        y1, y2 = (-A * x1 - C) / B, (-A * x2 - C) / B
    return [(x1, y1), (x2, y2)]


def process_raw_data():
    """ Simulates getting new data (replace this with real sensor data) """
    return {
        'leftVector': [(0, 0), (np.random.uniform(1, 3), np.random.uniform(1, 3))],
        'rightVector': [(1, 0), (np.random.uniform(2, 4), np.random.uniform(2, 4))],
        'finish_line_left_segment': [(3, 0), (3, 2)],
        'finish_line_right_segment': [(4, 0), (4, 2)],
        'leftLine': (1, -1, -np.random.uniform(1, 5)),
        'rightLine': (-1, 2, np.random.uniform(1, 5)),
        'middleLaneLine': (0, 1, -np.random.uniform(1, 5)),
    }


def update_plot(data_dict, ax):
    ax.clear()

    left_vector = data_dict['leftVector']
    right_vector = data_dict['rightVector']
    ax.plot([left_vector[0][0], left_vector[1][0]], [left_vector[0][1], left_vector[1][1]], 'b-', label='Left Vector')
    ax.plot([right_vector[0][0], right_vector[1][0]], [right_vector[0][1], right_vector[1][1]], 'r-',
            label='Right Vector')

    fl_left = data_dict['finish_line_left_segment']
    fl_right = data_dict['finish_line_right_segment']
    ax.plot([fl_left[0][0], fl_left[1][0]], [fl_left[0][1], fl_left[1][1]], 'g--', label='Finish Line Left')
    ax.plot([fl_right[0][0], fl_right[1][0]], [fl_right[0][1], fl_right[1][1]], 'm--', label='Finish Line Right')

    for line, color, name in zip(
            ['leftLine', 'rightLine', 'middleLaneLine'],
            ['orange', 'cyan', 'purple'],
            ['Left Line', 'Right Line', 'Middle Lane Line']
    ):
        points = line_to_points(data_dict[line])
        ax.plot([points[0][0], points[1][0]], [points[0][1], points[1][1]], color=color, linestyle='-', label=name)

    ax.set_title("Live Update: Vectors & Finish Lines")
    ax.legend()
    ax.set_xlabel("X Coordinate")
    ax.set_ylabel("Y Coordinate")
    #ax.set_xlim(-5, 5)
    #ax.set_ylim(-5, 5)

    plt.draw()