from numpy import random
from numpy.random import normal as rndnorm
def gen_white_noize(std, sample_time, data_frequency):
    mean = 0
    num_samples = sample_time*data_frequency
    samples = rndnorm(mean, std, size=num_samples)
    return samples