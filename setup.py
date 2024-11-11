from setuptools import setup
from Cython.Build import cythonize

setup(name='cheetahgym',
      version='0.0.1',
      install_requires=['gym','torch','pybullet','matplotlib', 'numpy_indexed', 'open3d', 'tabulate'],#'opencv_python']  # And any other dependencies package needs
      ext_modules = cythonize('cheetahgym/utils/mpc_optimization_function.pyx'),
)
