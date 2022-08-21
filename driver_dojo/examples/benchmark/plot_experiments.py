import glob
import os
import tensorflow as tf
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
sns.set()

class experiment_1:
    algo = 'ppo'
    act = ['disc', 'sem']
    map = 'intersection'
    difficulties = ['traffic_plus_dens_vTypes', 'traffic_plus_dens', 'traffic_plus', 'simple']
    sample_sizes = [1, 3, 10, 50, 100, 200]#, 1000]
    metrics = ['reward', 'goal']

from tensorflow.core.util import event_pb2
from tensorflow.python.lib.io import tf_record

def my_summary_iterator(path):
    for r in tf_record.tf_record_iterator(path):
        yield event_pb2.Event.FromString(r)

def read_from_events(path):
    summary = {}
    for event in my_summary_iterator(path):
        for value in event.summary.value:
            #t = tensor_util.MakeNdarray(value.tensor)
            #print(value.tag, event.step, t, type(t))
            if value.tag not in summary:
                summary[value.tag] = {}
            summary[value.tag][event.step] = value.simple_value

base_path = 'outputs'
def plot_experiment1():
    # Setup plot data
    plot_data = {}
    for act in experiment_1.act:
        plot_data[act] = {}
        for difficulty in experiment_1.difficulties:
            plot_data[act][difficulty] = {}
            for split in ['train', 'test']:
                plot_data[act][difficulty][split] = {}
                for metric in experiment_1.metrics:
                    plot_data[act][difficulty][split][metric] = []

    for act in experiment_1.act:
        for difficulty in experiment_1.difficulties:
            for sample_size in experiment_1.sample_sizes:
                exp_output = f"{experiment_1.algo}_{act}_{experiment_1.map}_{difficulty}_{sample_size}"
                event_path = glob.glob(os.path.join('outputs', exp_output, 'event*'))[0]

                summary = read_from_events(event_path)
                # Find last test time_step
                last_test = max([int(x) for x in summary['test/reward'].keys()])
                train_steps = [int(x) for x in summary['train/reward'].keys()]
                nearest_train = np.argmin(
                    np.abs(np.array(train_steps) - last_test)
                )
                for metric in experiment_1.metrics:
                    plot_data[act][difficulty]['train'][metric] = summary[f'train/{metric}'][nearest_train]
                    plot_data[act][difficulty]['test'][metric] = summary[f'test/{metric}'][last_test]

    print(plot_data)


plot_experiment1()