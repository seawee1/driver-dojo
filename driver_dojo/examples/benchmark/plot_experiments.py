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
    #difficulties = ['traffic_plus_dense_vType', 'traffic_plus_dens', 'traffic_plus', 'simple']
    difficulties = ['simple', 'traffic_plus', 'traffic_plus_dens', 'traffic_plus_dense_vType']
    sample_sizes = [3, 10, 50, 100, 200, 1000]
    metrics = ['reward', 'goal', 'collision']

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
    return summary

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
                    plot_data[act][difficulty][split][metric] = {}

    for act in experiment_1.act:
        for difficulty in experiment_1.difficulties:
            for sample_size in experiment_1.sample_sizes:
                exp_output = f"{experiment_1.algo}_{act}_{experiment_1.map}_{sample_size}_{difficulty}"
                print(exp_output)
                event_path = glob.glob(os.path.join('outputs', exp_output, 'output', 'event*'))
                if len(event_path) == 0:
                    if difficulty == 'traffic_plus_dens':
                        exp_output = f"{experiment_1.algo}_{act}_{experiment_1.map}_{sample_size}_{'traffic_plus_dense'}"
                        event_path = glob.glob(os.path.join('outputs', exp_output, 'output', 'event*'))
                    elif difficulty == 'traffic_plus':
                        exp_output = f"{experiment_1.algo}_{act}_{experiment_1.map}_{sample_size}_{'traffic_pluss'}"
                        event_path = glob.glob(os.path.join('outputs', exp_output, 'output', 'event*'))
                    elif difficulty == 'traffic_plus_dense_vType':
                        exp_output = f"{experiment_1.algo}_{act}_{experiment_1.map}_{sample_size}_{'traffic_plus_dens_vType'}"
                        event_path = glob.glob(os.path.join('outputs', exp_output, 'output', 'event*'))
                    else:
                        print("No events file")
                        continue
                event_path = event_path[0]

                summary = read_from_events(event_path)
                # Find last test time_step
                for metric in experiment_1.metrics:
                    last_test = max([int(x) for x in summary[f'test/{metric}'].keys()])
                    train_steps = [int(x) for x in summary[f'train/{metric}'].keys()]
                    nearest_train = train_steps[np.argmin(
                        np.abs(np.array(train_steps) - last_test)
                    )]
                    plot_data[act][difficulty]['train'][metric][sample_size] = (summary[f'train/{metric}'][nearest_train])
                    plot_data[act][difficulty]['test'][metric][sample_size] = (summary[f'test/{metric}'][last_test])

    sup = ['Discrete Act., Intersection', 'Semantic Act., Intersection']
    dif = ['Simple', '+Traffic', '+Density', '+vTypes']
    me = ['Reward', 'Goal rate', 'Collision rate']
    for k, act in enumerate(experiment_1.act):
        fig, axs = plt.subplots(len(experiment_1.difficulties), len(experiment_1.metrics))
        plt.suptitle(sup[k])
        for i, difficulty in enumerate(experiment_1.difficulties):
            for j, metric in enumerate(experiment_1.metrics):
                if j==0:
                    axs[i, j].set_ylabel(dif[i])
                if i==3:
                    axs[i, j].set_xlabel(me[j])

                axs[i, j].plot(plot_data[act][difficulty]['train'][metric].keys(), plot_data[act][difficulty]['train'][metric].values(),
                               label='Train', marker='*')
                axs[i, j].plot(plot_data[act][difficulty]['test'][metric].keys(), plot_data[act][difficulty]['test'][metric].values(),
                               label='Test', marker='*')

                axs[i, j].set_xticks(experiment_1.sample_sizes)
                axs[i, j].set_xscale('log')


        plt.legend()
        plt.savefig(f"{act}.png", bbox_inches='tight', dpi=100)
        plt.show()

if __name__ == '__main__':
    plot_experiment1()