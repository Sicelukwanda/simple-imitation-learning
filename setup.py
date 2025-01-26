from setuptools import setup, find_packages

setup(
    name="imitation_learning",
    version="0.1.0",
    description="A Python package for imitation learning using PyTorch and PyBullet.",
    author="Sicelukwanda Zwane",
    author_email="sicelukwanda.zwane@gmail.com",
    packages=find_packages(),  # Automatically find all packages in the directory
    include_package_data=True,  # Include package data specified in MANIFEST.in
    package_data={
        "imitation_learning": [
            "robot_models/mycobot_description/**/*",  # Include all files under mycobot_description
        ],
    },
    install_requires=[
        "numpy",
        "pybullet",
        "torch",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
)
