import codecs
import os
from pathlib import Path

import setuptools


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), "r") as fp:
        return fp.read()


def get_version(rel_path):
    for line in read(rel_path).splitlines():
        if line.startswith("__version__"):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError("Unable to find version string.")


core_requirements = [
    "gymnasium>=1.0.0",
    # pyquaternion doesn't support 2.x yet
    "numpy==1.26.*",
    "safetensors==0.6.2",
    # WARNING: recorded demos might break when updating Mujoco
    "mujoco==3.1.5",
    # needed for pyMJCF
    "dm_control==1.0.19",
    "imageio",
    "pyquaternion",
    "mujoco_utils",
    "wget",
    "mojo-mujoco-wrapper>=0.1.1",
    "pyyaml",
    "dearpygui",
    "pyopenxr<1.1.5001",
]

setuptools.setup(
    version=get_version("bigym/__init__.py"),
    name="bigym",
    author="Nikita Cherniadev",
    author_email="nikita.chernyadev@gmail.com",
    packages=setuptools.find_packages(),
    python_requires=">=3.10",
    install_requires=core_requirements,
    package_data={
        "": [str(p.resolve()) for p in Path("bigym/envs/xmls").glob("**/*")]
        + [str(p.resolve()) for p in Path("bigym/envs/presets").glob("**/*.yaml")]
        + [str(p.resolve()) for p in Path("vr/viewer/xmls").glob("**/*")]
    },
    extras_require={
        "dev": ["pre-commit", "pytest"],
        "examples": [
            "moviepy",
            "pygame",
            "opencv-python",
            "matplotlib",
        ],
    },
)
