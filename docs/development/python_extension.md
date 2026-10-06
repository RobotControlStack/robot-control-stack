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

3.  **Register the Backend**: If your extension provides a camera, gripper or hand, declare a factory
    as an entry point so the robot extensions can create it from a config. The entry point name is
    the type id the config carries (`camera_type_id`, `GripperType.id` or `HandType.id`), and the
    group is one of `rcs.cameras`, `rcs.grippers` or `rcs.hands`:

    ```toml
    [project.entry-points."rcs.cameras"]
    mycam = "rcs_myext.creators:create_camera_set"
    ```

    The factory takes the config and returns the device, for a camera
    `(HardwareCameraCreatorConfig) -> HardwareCamera`. Nothing else has to know about your
    extension: `rcs.registry` lists installed backends from package metadata without importing
    them, and imports yours only when its id is requested. Reinstall the extension after changing
    entry points, they are read from the installed metadata. For code that is not installed as a
    package, `rcs.registry.CAMERAS.register("mycam", create_camera_set)` does the same at runtime.

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
