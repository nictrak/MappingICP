import numpy as np

FILE = 'map'


def save_override_map(data):
    print("...saving")
    np.save(FILE, data)
    print("save complete")


def load_map():
    np.load(FILE)
    return np.array(np.load(FILE))




