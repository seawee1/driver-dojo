from setuptools import setup
from setuptools import find_packages

setup(
    name="driver_dojo",
    version="0.1.0",
    python_requires=">=3.8",
    install_requires=[
        "gym==0.21.0",
        # "eclipse-sumo==1.13.0",  # Better to install it manually as we encountered some problems with pip installations.
        "traci==1.13.0",
        "sumolib==1.13.0",
        "matplotlib",
        "scikit-image",
        "shapely==1.8.1.post1",
        "pyyaml",
        "numpy==1.21.0",
        "scipy==1.8.0",
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
