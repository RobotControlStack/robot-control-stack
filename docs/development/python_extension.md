# Creating a Python Extension

Creating a Python-based extension is the easiest way to add new functionality to RCS, especially for hardware that already has a Python API.

## Structure

A typical Python extension has the following structure:

```text
rcs_myext/
├── pyproject.toml
├── README.md
└── src/
    └── rcs_myext/
        ├── __init__.py
        └── my_device.py
```

## Steps

1.  **Create `pyproject.toml`**: Define your package metadata and dependencies.

    ```toml
    [build-system]
    requires = ["setuptools>=61.0"]
    build-backend = "setuptools.build_meta"

    [project]
    name = "rcs_myext"
    version = "0.1.0"
    dependencies = [
        "rcs",
        # Add other dependencies here
    ]
    ```

2.  **Implement the Interface**: Create your device class in `src/rcs_myext/my_device.py`. You should inherit from the appropriate RCS base class (e.g., `Camera`, `Gripper`) if applicable, or implement the required methods.

3.  **Register the Extension**: If your extension needs to be discoverable by RCS (e.g., for CLI tools or automatic loading), ensure it's installed in the same environment.

## Example: USB Camera

The `rcs_usb_cam` extension is a good example of a pure Python extension. It wraps `cv2.VideoCapture` to provide a camera interface compatible with RCS.

```python
from rcs.camera import Camera

class WebCam(Camera):
    def __init__(self, device_id=0):
        # ... initialization ...
        pass

    def get_image(self):
        # ... capture frame ...
        return image
```
