# Documentation

This directory contains the source code for the Robot Control Stack (RCS) documentation.

## Building Locally

To build the documentation locally, follow these steps:

1.  **Install Dependencies**:
    Ensure you have the documentation dependencies installed.

    ```shell
    pip install -r requirements.txt
    ```

2.  **Build**:
    Run `sphinx-build` to generate the HTML documentation.

    ```shell
    sphinx-build -b html . _build/html
    ```

3.  **View**:
    Open `_build/html/index.html` in your web browser.

## Live Reloading

For a better development experience, you can use `sphinx-autobuild` to automatically rebuild the documentation when you make changes.

```shell
sphinx-autobuild . _build/html
```

This will start a local server (usually at http://127.0.0.1:8000) and refresh the page whenever you save a file.
