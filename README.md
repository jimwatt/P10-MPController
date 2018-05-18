# Model Predictive Control

This project is in conjunction with the Udacity Self-Driving Car course.  In particular, the goal of this project is to implement a simple model predictive controller to safely throttle and steer a simulated car around a track. 

* More detail about the approach is provided in [writeup.md](./writeup.md).

## Getting Started

### Prerequisites

```
CMake
uWS		(Socket for interfacing with Udacity simulator)	
```

### Installing

1. Clone this project from GitHub:

```
git clone https://github.com/jimwatt/P10-MPController.git
```

2. Change into the `build` directory, and compile with `CMake`.

   ```
   cd build
   cmake ..
   make
   ```

## Running the Code

* Run the executable from the `build` directory:
  ```./mpc```
* The executable will wait until measurements are provided by the simulator through the uWS socket.





## Authors

* **James Watt**

<!--## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details-->

## Acknowledgments
This project is a submission to the Udacity Self-Driving Car nanodegree:

* ```
  https://github.com/udacity/CarND-MPC-Control-Project.git
  ```

