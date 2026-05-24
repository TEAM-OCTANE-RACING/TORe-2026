# ft-fsd-path-planning

Formula Student Driverless Path Planning Algorithm

The algorithm requires the following inputs:

- The car's current position and orientation in the slam map
- The position of the (optionally colored) cones in the slam map

The algorithm outputs:

- Samples of a parameterized b-spline with the x,y and curvature of the samples

The algorithm is completely stateless. Every time it is called no previous results are
used. The only aspect that can be used again is the path that was previously generated.
It is only used if the path calculation has failed.

The parts of the pipeline are also available as individual classes, so if you only
want to use parts of it you can do so.

The codebase is written entirely in Python and makes heavy use of NumPy, SciPy, and Numba.

The algorithm has demonstrated its success as part of the FaSTTUBe pipeline, contributing to a 2nd place finish in Trackdrive at FS Czech 2023.

## Installation

The package can be installed using pip:

```bash
pip install "fsd-path-planning[demo] @ git+https://git@github.com/papalotis/ft-fsd-path-planning.git"
```

This will also install the dependencies needed to run the demo (cli, matplotlib, streamlit, etc.). If you don't want to install the demo dependencies, you can install the package without the `demo` extra:

```bash
pip install "fsd-path-planning @ git+https://git@github.com/papalotis/ft-fsd-path-planning.git"
```

You can also clone the repository and install the package locally:

```bash
git clone https://github.com/papalotis/ft-fsd-path-planning.git
cd ft-fsd-path-planning
pip install -e .[demo]
```

You can again skip the `[demo]` extra if you don't want to install the demo dependencies.

## Performance

The algorithm (with default parameters) is fast enough to run in real-time on a Jetson Xavier AGX 16GB on MAXN power mode. On that platform, the algorithm takes on average around 10ms from start to finish. You can run the demo to get an idea of the performance on your hardware.

*Note that the first time that you run the algorithm, it will take around 30-60 seconds to compile all the Numba functions. Run the demo a second time to get a real indicator on performance.*

Run the following command to run the demo on your machine:

```bash
python -m fsd_path_planning.demo
```

## Basic usage

```python
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes

path_planner = PathPlanner(MissionTypes.trackdrive)
# you have to load/get the data, this is just an example
global_cones, car_position, car_direction = load_data() 
# global_cones is a sequence that contains 5 numpy arrays with shape (N, 2),
# where N is the number of cones of that type

# ConeTypes is an enum that contains the following values:
# ConeTypes.UNKNOWN which maps to index 0
# ConeTypes.RIGHT/ConeTypes.YELLOW which maps to index 1
# ConeTypes.LEFT/ConeTypes.BLUE which maps to index 2
# ConeTypes.START_FINISH_AREA/ConeTypes.ORANGE_SMALL which maps to index 3
# ConeTypes.START_FINISH_LINE/ConeTypes.ORANGE_BIG which maps to index 4

# car_position is a 2D numpy array with shape (2,)
# car_direction is a 2D numpy array with shape (2,) representing the car's direction vector
# car_direction can also be a float representing the car's direction in radians

path = path_planner.calculate_path_in_global_frame(global_cones, car_position, car_direction)

# path is a Mx4 numpy array, where M is the number of points in the path
# the columns represent the spline parameter (distance along path), x, y and path curvature

```

Take a look at this notebook for a more detailed example: [simple_application.ipynb](fsd_path_planning/demo/simple_application.ipynb)

> [!TIP]
> There is no resetting functionality in the classes. If you want to reset the path planner, you can simply create a new instance of the class.
It is recommended to create a new instance of the relevant classes when the vehicle enters `AS-READY` state.

## Previous versions

Alternate versions of the algorithm are available as git tags:

- `color-dependent` - The algorithm needs color information to work. This version was used in the 2021/22 season.
- `summer-23` - The algorithm can work without color information. This version was used in the 2022/23 season.
