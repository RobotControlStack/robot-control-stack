# RCS SO101 Extension

## Package Dependency Issue Workaround
The extension for SO101 depends on the `lerobot` package, which depends on `opencv-python-headless`, which uninstalls RCS' own `opencv-python` dependency install. 
Since the extension doesn't care about the OpenCV-dependent parts of `lerobot`, this package works fine even with the full `opencv-python`, but it needs to be re-installed manually.
You can do it by the following: 
```
pip uninstall opencv-python-headless
pip uninstall opencv-python
pip install opencv-python~=4.10.0.84
```
which will reinstall the correct version of OpenCV to the environment. 
Unfortunately, there's no easy automated solution for it, so it needs to be done manually each time this extension is installed with `pip install -e .`.