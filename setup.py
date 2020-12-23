from setuptools import setup, find_packages

# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import ParallelCompile, Pybind11Extension, build_ext
from pathlib import Path

# Optional multithreaded build
__version__ = "0.0.1"
map_path = (Path(__file__).parent / Path('carla_utils/map_cpp')).absolute()

ext_modules = [
    Pybind11Extension("carla_utils.map",
                      sorted([str(p) for p in map_path.rglob('*.cpp')]),
                      cxx_std=17, include_dirs=[str(map_path)]
                      ),
]

ParallelCompile("NPY_NUM_BUILD_JOBS").install()

setup(
    name="carla_utils",
    version=__version__,
    author="UT Deep learning group",
    author_email='philkr@cs.utexas.edu',
    url="https://github.com/philkr/carla_utils",
    description='Extra features for CARLA',
    long_description="",
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Scientific/Engineering',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    license='MIT',
    packages=find_packages(include=['carla_utils', 'carla_utils.*']),
    ext_modules=ext_modules,
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
)
