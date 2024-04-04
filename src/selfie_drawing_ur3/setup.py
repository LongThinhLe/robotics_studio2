from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["image_processor", "ur3_control", "GUI_lib"],
    package_dir={"": "src"}
)

setup(**d)