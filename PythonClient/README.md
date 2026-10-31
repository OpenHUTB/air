# Python API for AirSim

This package contains Python APIs for [AirSim](https://github.com/microsoft/airsim).

## How to Use
See examples at [car/hello_car.py](https://github.com/Microsoft/AirSim/blob/main/PythonClient/car/hello_car.py) or [multirotor/hello_drone.py](https://github.com/microsoft/AirSim/blob/main/PythonClient/multirotor/hello_drone.py).

## Dependencies
This package depends on `msgpack` and would automatically install `msgpack-rpc-python` (this may need administrator/sudo prompt):
```
pip install msgpack-rpc-python
```

Some examples also requires opencv.

## Tests

The client helpers have offline unit tests that need neither Unreal Engine nor a
running simulator:

```bash
cd PythonClient
python -m unittest discover -s tests -v
```

## More Info

More information on AirSim Python APIs can be found at:
https://github.com/Microsoft/AirSim/blob/main/docs/python.md

