# SIMPLE WAY TO SHOW THE GENERAL INFORMATION OF OUR PACKAGE WITH PIP

import os

# General package "fdhc" info
print("\n-----GENERAL FDHC INFORMATION-----")
os.system("pip show fdhc")

# Specific package "fdhc" dependency-tree info
# (must have installed pipdeptree)
print("\n-----SPECIFIC FDHC INFORMATION-----")
os.system("pipdeptree -p fdhc")

# -----SPECIFIC FDHC INFORMATION-----
# fdhc==0.0.1
#   - matplotlib [required: Any, installed: 2.2.5]
#     - backports.functools-lru-cache [required: Any, installed: 1.6.1]
#     - cycler [required: >=0.10, installed: 0.10.0]
#       - six [required: Any, installed: 1.15.0]
#     - kiwisolver [required: >=1.0.1, installed: 1.1.0]
#       - setuptools [required: Any, installed: 44.1.1]
#     - numpy [required: >=1.7.1, installed: 1.16.6]
#     - pyparsing [required: >=2.0.1,!=2.1.6,!=2.1.2,!=2.0.4, installed: 2.4.7]
#     - python-dateutil [required: >=2.1, installed: 2.8.1]
#       - six [required: >=1.5, installed: 1.15.0]
#     - pytz [required: Any, installed: 2021.1]
#     - six [required: >=1.10, installed: 1.15.0]
#   - numpy [required: Any, installed: 1.16.6]
#   - opencv-python [required: ==4.2.0.32, installed: 4.2.0.32]
#     - numpy [required: >=1.11.1, installed: 1.16.6]
