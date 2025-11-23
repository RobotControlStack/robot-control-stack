# Contributing

We welcome contributions to the Robot Control Stack!

## Development Setup

1.  Clone the repository.
2.  Install dependencies (see [Getting Started](../getting_started/index.md)).
3.  Install pre-commit hooks:
    ```shell
    pre-commit install
    ```

## Development Tools

We provide a `Makefile` with several useful commands for development.

### Python

- **Formatting**:
  ```shell
  make pyformat
  ```
  Uses `isort` and `black` to format code.

- **Linting**:
  ```shell
  make pylint
  ```
  Runs `ruff` and `mypy`.

- **Testing**:
  ```shell
  make pytest
  ```
  Runs the test suite.

- **Type Stubs**:
  ```shell
  make stubgen
  ```
  Generates Python type stubs for the C++ bindings.

### C++

- **Formatting**:
  ```shell
  make cppformat
  ```
  Uses `clang-format` to format C++ code.

- **Linting**:
  ```shell
  make cpplint
  ```
  Runs `clang-tidy`.

### General

- **Commit**:
  ```shell
  make commit
  ```
  Uses `commitizen` to help you create conventional commits.

## Code Style


- **Python**: We use `ruff` for linting and formatting.
- **C++**: We use `clang-format` and `clang-tidy`.

## Commit Messages

We follow the **Conventional Commits** specification. This allows us to automatically generate changelogs and determine semantic versioning.

**Format**: `<type>[optional scope]: <description>`

**Types**:
- `feat`: A new feature
- `fix`: A bug fix
- `docs`: Documentation only changes
- `style`: Changes that do not affect the meaning of the code (white-space, formatting, etc)
- `refactor`: A code change that neither fixes a bug nor adds a feature
- `perf`: A code change that improves performance
- `test`: Adding missing tests or correcting existing tests
- `build`: Changes that affect the build system or external dependencies
- `ci`: Changes to our CI configuration files and scripts
- `chore`: Other changes that don't modify src or test files

**Examples**:
- `feat(fr3): add support for new gripper`
- `fix(sim): fix collision detection bug`
- `docs: update installation instructions`

See [conventionalcommits.org](https://www.conventionalcommits.org/) for more details.

## Pull Requests

1.  Create a new branch for your feature or fix.
2.  Make your changes and commit them using the conventional commit format.
3.  Push your branch and open a Pull Request.
4.  Ensure all CI checks pass.
