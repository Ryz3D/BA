import numpy as np

def preprocess(full_data_y):
    full_data_y[1] *= -1
    full_data_y[0:3] *= 40 * 9.81
    full_data_y[3:6] *= 100
    return full_data_y
