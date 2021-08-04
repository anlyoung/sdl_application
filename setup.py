from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['sdl_application', 'sdl_dataflow', 'sdl_interface'],
    package_dir={'': 'src'}
)
setup(**d)