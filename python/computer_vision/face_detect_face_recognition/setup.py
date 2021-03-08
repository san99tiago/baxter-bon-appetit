from setuptools import setup, find_packages

# Get README.md long description to add it to the builded package
with open("README.md", "r") as fh:
    long_description = fh.read()

setup(

    # Library name (same as "pip install ____")
    name='fdfr',

    author='Santiago Garcia and Elkin Guerra',

    # Package version (MAJOR_VERSION.MINOR_VERSION.MAINTENANCE_VERSION)
    version='0.0.1',

    url="https://github.com/san99tiago/tesis",

    # Simple package description
    description='Face recognition library implemented with examples',

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
        "dlib==19.21.1",
        "face-recognition==1.3.0",
    ],

    # Add devlopment dependencies
    extras_require={
        "dev": [
            "pytest",
        ],
    },

    classifiers=[
        "Programming Language :: Python :: 2.7",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Development Status :: 3 - Alpha"
    ],

    license="MIT License",

    # Add extra XML and JSONs needed
    include_package_data=True
)
