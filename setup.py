from setuptools import setup
from setuptools import find_packages

setup(
    name="driver_dojo",
    version="0.1.0",
    python_requires=">=3.8",
    install_requires=[
        "gym",
        #"eclipse-sumo==1.15.0",  # Better to install it manually as we encountered some problems with pip installations.
        "traci==1.15.0",
        "sumolib==1.15.0",
        "matplotlib",
        "scikit-image",
        "shapely==1.8.1.post1",
        "pyyaml",
        "scipy",
        "rtree",
        "pyclothoids==0.1.4",
        "scenariogeneration",
        "psutil",
        "omegaconf",
        "ffmpeg-python",
        #"carla",
        "lxml",
        "visdom",
        "commonroad-vehicle-models",
        "pyglet==1.5.27"
    ],
    # extras_require={
    #     "benchmark": [
    #         "tianshou",
    #         "hydra-core",
    #         "torch",
    #         "torchvision",
    #         "torchaudio",
    #         "tensorflow",
    #         "tensorboard",
    #         "pandas",
    #         "seaborn",
    #     ],
    # },
    packages=find_packages(
        where="", include=["driver_dojo"], exclude=["data", "media", "tools"],
    ),
    author="Sebastian Rietsch",
)
