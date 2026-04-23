# Remote Inference

RCS includes a lightweight RPC (Remote Procedure Call) layer based on [RPyC](https://rpyc.readthedocs.io/). This allows you to run an RCS environment on one machine (e.g., a machine connected to robot hardware) and control it or run inference from another machine (e.g., a powerful GPU server).

## Overview

The RPC system consists of two main components:
- **`RcsServer`**: Wraps an existing RCS environment and exposes it over the network.
- **`RcsClient`**: A Gymnasium-compatible environment that forwards all calls to a remote `RcsServer`.

## Usage

### Starting the Server

The server machine should be the one physically connected to the robot or running the simulation.

```python
from rcs.envs.configs import EmptyWorldFR3
from rcs.rpc.server import RcsServer

# Create your environment
scene = EmptyWorldFR3()
env = scene.create_env(scene.config())

# Start the RPC server
server = RcsServer(env, port=50051)
server.start()
```

### Connecting with the Client

The client machine runs your control logic or neural network inference.

```python
from rcs.rpc.client import RcsClient

# Connect to the remote server
client = RcsClient(host="robot-machine-ip", port=50051)

# Use it like a local Gymnasium environment
obs, info = client.reset()
action = model.predict(obs)
obs, reward, terminated, truncated, info = client.step(action)
```

## Advantages

- **Hardware Isolation**: Keep your expensive GPU server away from the robot workspace.
- **Resource Management**: Run heavy inference on a dedicated machine while the robot control machine handles low-level loops.
- **Flexibility**: Easily switch between local and remote environments by just changing the environment initialization.

## Examples

See `examples/rpc_server_client/` for a complete working example of a server and client setup.
