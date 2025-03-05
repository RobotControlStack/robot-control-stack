# Usage

The python package is called `rcsss` (sss because of the sound of a snake).
Import the library in python:

```
import rcsss
```

The package includes a command line interface which define useful commands to handle the hardware robot.
To list all available subcommands use:

``` 
python -m rcsss --help
```
A sample config can be generated via the following CLI command.

``` 
python -m rcsss sample-config
```
The command will produce a `config.yaml` file with sample values.
See [config.py](https://github.com/utn-mi/robot-control-stack/blob/master/python/rcsss/config.py) for a description of the config fields.