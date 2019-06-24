from setuptools import find_packages
from setuptools import setup

package_name = 'etddf_ros2'

setup(
    name=package_name,
    version='0.7.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    package_dir={package_name: 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/', ['launch/agent.launch.py']),
        ('share/' + package_name + '/', ['config/ros_agent_config.yaml']),
        ('share/' + package_name + '/', ['config/points.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Ian Loefgren',
    author_email='ian.loefgren@colorado.edu',
    maintainer='Ian Loefgren',
    maintainer_email='ian.loefgren@colorado.edu',
    url='https://github.com/COHRINT/et-ddf',
    download_url='https://github.com/COHRINT/et-ddf/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Event-triggered decentralized data fusion.',
    long_description="""\
ROS packaging of event-triggered decentralized data fusion python package.""",
    # license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_wrapper = src.agent_wrapper:main',
            'comms = src.comms:main'
        ]
    }
)
