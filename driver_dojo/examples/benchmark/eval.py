import os
import hydra
import sys
import argparse
from omegaconf import OmegaConf, open_dict
from driver_dojo.examples.benchmark.benchmark import benchmark
from hydra.core.hydra_config import HydraConfig
from glob import glob
from pathlib import Path


def eval(args, output_path):
    wd = os.getcwd()
    with hydra.initialize(version_base=None, config_path="experiments", job_name='eval'):
        os.chdir(output_path)
        override_config = OmegaConf.load(os.path.join('.hydra', 'overrides.yaml'))
        config = hydra.compose(
            config_name="config",
            overrides=override_config,
        )

        if args.train:
            config.env_test = config.env_train
        with open_dict(config) as config:
            config.eval = True
            config.eval_file = 'train_results.yaml' if args.train else 'test_results.yaml'
        config.algo.params.test_num = args.num
        config.algo.params.training_num = None

        benchmark(config)

    os.chdir(wd)


def eval_recursive(args, base_path):
    import random
    output_paths = [x for x in glob(os.path.join(base_path, args.pattern))]
    todo = [i for i in range(len(output_paths))]
    while len(todo) > 0:
        idx = random.choice(todo)
        output_path = output_paths[idx]

        eval_file = 'train_results.yaml' if args.train else 'test_results.yaml'
        lock_file = os.path.join(output_path, '.eval_lock')
        if os.path.isfile(os.path.join(output_path, 'output', eval_file)) \
                or os.path.isfile(lock_file):
            todo.remove(idx)
            continue

        print(output_path)
        with open(lock_file, 'w') as f:
            f.write('Yo, waddup')

        import multiprocessing
        p = multiprocessing.Process(target=eval, args=[args, output_path])
        p.start()
        p.join()
        p.terminate()
        todo.remove(idx)

        try:
            os.remove(lock_file)
        except Exception:
            pass

        print(f'Eval finished, {len(todo)} models to go...')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Evaluate')
    parser.add_argument('output_path')
    parser.add_argument('--num', default=1, type=int)
    parser.add_argument('--train', action='store_true')
    parser.add_argument('--recursive', action='store_true')
    parser.add_argument('--pattern', default='*', type=str)
    args = parser.parse_args()

    if args.recursive:
        eval_recursive(args, args.output_path)
    else:
        eval(args, args.output_path)
