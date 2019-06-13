#!/usr/bin/env python

from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup

setup(
    name='etddf',
    description='event-triggered decentralized data fusion package',
    author='Ian Loefgren',
    author_email='ian.loefgren@colorado.edu',
    packages=['etddf','etddf.filters']
)

# setup_args = generate_distutils_setup(
#      packages=['offset'],
#      package_dir={'': 'src'}
# )

# setup(**setup_args)
