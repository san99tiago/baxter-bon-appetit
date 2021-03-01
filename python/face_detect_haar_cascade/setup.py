from setuptools import setup, find_packages

# Get README.md long description to add it to the builded package
with open("README.md", "r") as fh:
    long_description = fh.read()

setup(

    # Library name (same as "pip install ____")
    name='fdhc',

    author='Santiago Garcia and Elkin Guerra',

    # Package version (MAJOR_VERSION.MINOR_VERSION.MAINTENANCE_VERSION)
    version='0.0.1',

    # Simple package description
    description='Face detection library implemented with examples',

    # Long package description
    long_description=long_description,
    long_description_content_type="text/markdown",

    # Our main folder to build the package
    package_dir={'': 'src'},

    # Add library dependencies
    install_requires=[
        "numpy",
        "matplotlib",
        "opencv-python==4.2.0.32",
    ],

    # Add devlopment dependencies
    extras_require={
        "dev": [
            "pytest",
        ],
    },

    # Add extra XML and JSONs needed
    include_package_data=True
)
