# Driver Dojo: A Benchmark for Generalizable Reinforcement Learning for Autonomous Driving
![logo](./media/logo/logo.gif)


Driver Dojo is an autonomous driving environment for benchmarking reinforcement learning (RL) methods based on the popular [Simulation of Urban Mobility (SUMO)](https://www.eclipse.org/sumo/) platform. 
It focuses on generalizability, meaning that we aim to provide means to develop, prototype, research and compare RL methods that are able to handle broad distributions of driving scenarios.
For this, we provide randomization on the street network level, the task level (ego routing), and traffic level, where the latter encompasses traffic density, traffic initialization and routing, as well as driving personalities through sampling of unique physical, car-following, lane-change and junction model parameters on a per-vehicle basis.

The key features of Driver Dojo are:
- **Generalization environments:** Five fine-tuned environments for intersection, roundabout and highway driving scenarios.
- **Pre-implemented observations:** Ten composable observers allowing for compact, ground-truth numerical observations extracted from the SUMO simulation state as well as image observations through a simplified birds-eye observer and co-simulation in [Carla](https://carla.org/).
- **Multiple action spaces:** A continuous, discretized as well as semantic action space founded on automatic lane-following through a low-level controller.
- **Different vehicle models:** Physical dynamics and simplified lane-following car models.
- **Script library:** Python scripts for e.g. manually driving an environment using keyboard inputs, map plotting and RL model benchmarking through [Tianshou](https://github.com/thu-ml/tianshou) and [Hydra](https://github.com/facebookresearch/hydra.git).
- **High Configurability:** Fully typed configuration capabilities from the outside, allowing for diverse driving scenario creation without having to touch any source-code.
- **Full SUMO support:** Any configuration or static scenario definition natively supported in SUMO is straightforward to integrate as an RL environment.
- **Clean and performant:** Our codebase is modular, clean and fully documented. We are able to train an PPO agent for 10 million time steps in less than 24 hours on our desktop workstation. Have a look at our [documentation](https://driver-dojo.readthedocs.io/en/latest/?)!

## Showcase
Something for the eye.
### Core Randomized Scenarios
Have a look into [this folder](media/showcase-videos/) for more videos.
#### Sem-TPS-Intersection-v0 
![Sem-TPS-Intersection-v0](media/showcase-videos/intersection-showcase.gif)
#### Sem-TPS-Roundabout-v0 
![Sem-TPS-Intersection-v0](media/showcase-videos/roundabout-showcase.gif)
#### Sem-TPS-HighwayEntry-v0 
![Sem-TPS-Intersection-v0](media/showcase-videos/highwayentry-showcase.gif)

### Custom Scenarios
#### Sem-TPS-Urban_Intersection-v0
This scenario was built just using SUMO's netedit tool.
![Sem-TPS-Intersection-v0](media/showcase-videos/urbanintersection-showcase.gif)
#### Sem-TPS-FraunhoferNOP-v0
Here, we imported an area around our Fraunhofer institute in Nuremberg from OSM using SUMO's tooling scripts and added custom traffic definitions using netedit.
![Sem-TPS-Intersection-v0](media/showcase-videos/fraunhofernop-showcase.gif)
## Setup
Installing Driver Dojo and its requirements is very easy and does not require any source compilation whatsoever. 

Note that Carla does not work on macOS in its current state. However, if you do not plan to use the co-simulation feature you can nevertheless use our environment on macOS.

### Steps
1. Install SUMO 1.13 located [here](https://github.com/metadriverse/metadrive).

2. Install Carla 0.9.13 located [here](https://carla.org/).

3. Install our Driver Dojo package via:
```bash
git clone https://github.com/seawee1/driver-dojo.git
cd driver-dojo
pip install -e .
```
or
```bash
pip install -e .[benchmark]
```
if you plan to use our benchmark scripts.

4. As a last step, place the SUMO tooling scripts found inside the [official repository](https://github.com/eclipse/sumo/tree/v1_13_0/tools) under ```driver-dojo/tools```, i.e.:
```bash
git clone https://github.com/eclipse/sumo.git
cp -r sumo/tools/* driver-dojo/tools/
```

## Usage
### Quickstart
The easiest way to test your installation is to run:

```bash
python -m driver_dojo.examples.performance_benchmark [env=...] [subconfig.value=...]
```

where you can additionally pass the name of the environment you would like to test out via ```env=...``` and as many configuration parameters as you would like, e.g. ```simulation.dt=0.1```. Have a look at the [config.py](driver_dojo/core/config.py) to get an overview.

To get a real impression of what our benchmark looks like, please run:

```bash
python -m driver_dojo.examples.manual_control [env=...] [subconfig.value=...]
```

which lets you drive the ego vehicle using your keyboard.

### General Usage
Our environments follow the [OpenAI Gym](https://github.com/openai/gym) interface. To use it as a training environment, follow the typical workflow, i.e.
```python
import gym
import driver_dojo

env = gym.make("DriverDojo/PlaceEnvNameHere-v0")
obs = env.reset()
...
```
Generally, environment names are structured as ```DriverDojo/{Action Space}-{Vehicle Model}-{Scenario}-v0```, e.g. ```DriverDojo/Sem-TPS-Intersection-v0``` or ```DriverDojo/Disc-KS-Roundabout-v0```.

You can pass custom configurations through a config dict, for example: 
```python
import gym
import driver_dojo

config = dict(
      simulation=dict(
        dt=0.05
      ),
      observations=dict(
        observers=[...]
      )
)
env = gym.make("DriverDojo/PlaceEnvNameHere-v0", config=config)
```

### Running the Benchmark Code
We use Tianshou and Hydra for our benchmark code base. Run:
```bash
python -m driver_dojo.examples.benchmark hydra.job.chdir=True algo=ppo_numeric obs=default env="DriverDojo/Sem-TPS-Intersection-v0"
```
to train PPO on the intersection environment. This will create a subfolder inside ```output``` in which Tensorboard logs and model checkpoints will be placed.
Similarly, as shown above, this allows us to overwrite every configurable parameter from the outside, e.g. ```++env_train.simulation.dt=0.05``` will change the simulation time resolution of the training environment to 50ms.

#### Reproducing our Paper Experiments
For the results of our paper, we used ```env_train=train_x_0``` and ```env_train=train_x_1```, where ```x``` is the number of scenario constellations used for the respective experiment. For seeding of the RL method, we used seeds ```1337``` and ```1338```.
## Third-party Components
Besides [SUMO](https://www.eclipse.org/sumo/) and [Carla](https://carla.org/), we thank the authors of [Hydra](https://github.com/facebookresearch/hydra), [Tianshou](https://github.com/thu-ml/tianshou), [scenariogeneration](https://github.com/pyoscx/scenariogeneration) and [commonroad-vehicle-models](https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models), without which this project in its current form would not have been possible.

##
If you use Driver Dojo in your own work, please cite:
```bash
@misc{https://doi.org/10.48550/arxiv.2207.11432,
  doi = {10.48550/ARXIV.2207.11432},
  url = {https://arxiv.org/abs/2207.11432},
  author = {Rietsch, Sebastian and Huang, Shih-Yuan and Kontes, Georgios and Plinge, Axel and Mutschler, Christopher},
  title = {Driver Dojo: A Benchmark for Generalizable Reinforcement Learning for Autonomous Driving},
  publisher = {arXiv},
  year = {2022},
}
```
## License
This project is licensed under the [License for Academic Use of Fraunhofer Software, Version 2.0](LICENSE.txt).

For licenses of source code of third-party Open-Source projects directly included inside our repository, have a look at the [3rd-party-licences.md](3rd-party-licenses.md) file.
