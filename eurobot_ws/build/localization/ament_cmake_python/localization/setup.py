from setuptools import find_packages
from setuptools import setup

setup(
    name='localization',
    version='0.0.0',
    packages=find_packages(
        include=('localization', 'localization.*')),
)
