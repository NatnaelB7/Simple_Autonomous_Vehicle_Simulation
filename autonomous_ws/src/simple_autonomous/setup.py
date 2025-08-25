from setuptools import find_packages, setup

package_name = 'simple_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natnael',
    maintainer_email='nbtakele@aggies.ncat.edu',
    description='Simple autonomous vehicle project with obstacle stop and avoidance',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'obstacle_stop = simple_autonomous.obstacle_stop:main',
        	'obstacle_avoid = simple_autonomous.obstacle_avoid:main',
        ],
    },
)



