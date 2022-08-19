from setuptools import setup
from setuptools import find_packages

setup(
    name="driver_dojo",
    version="0.1.0",
    python_requires=">=3.8",
    install_requires=[
        "gym==0.21.0",
        "eclipse-sumo==1.14.1",  # Better to install it manually as we encountered some problems with pip installations.
        "traci==1.14.1",
        "sumolib==1.14.1",
        "matplotlib",
        "scikit-image",
        "shapely==1.8.1.post1",
        "pyyaml",
        "numpy==1.21.0",
        "scipy",
        "rtree",
        "pyclothoids==0.1.4",
        "scenariogeneration",
        "psutil",
        "omegaconf",
        "ffmpeg-python"
    ],
    extras_require={
        "benchmark": [
            "tianshou==0.4.8",
            "hydra-core",
            "torch",
            "torchvision",
            "torchaudio",
            "tensorflow",
            "tensorboard",
            "pandas",
            "seaborn",
        ],
    },
    packages=find_packages(
        where="", include=["driver_dojo"], exclude=["data", "media", "tools"],
    ),
    author="Sebastian Rietsch",
)
