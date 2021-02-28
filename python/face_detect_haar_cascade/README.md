# How to Install face_detec_haar_cascade Package

## Creating our Virtual Environment

For correct functionalities and future scalability, we should be working in a virtual environment to isolate our project and run it correctly. We are working with "venv".

## Add Project Dependencies

Then, we must install the dependency requirements for the project. These can be found at the root of the project.

```bash
pip install -r requirements.txt
```

Then, on the terminal, we must go to this specific path (on the same level as "setup.py").

## Install Our Package

After that, we must pip-install our own custom package with two possibilities:

Important remark: the flag "-e" lets us work locally and apply changes to our source code, without having to manually install it always.

### 1. Production Install

This will allows us to use the package, but we won't be able to make changes or generate tests for future improvement.

```bash
pip install -e .
```

### 2. Development Install

This will allow us to use the package and test it correctly for improvements.

```bash
pip install -e .[dev]
```

- Development dependencies are specified in the "setup.py".

## Authors

- Santiago Garcia Arango
- Elkin Javier Guerra Galeano

---