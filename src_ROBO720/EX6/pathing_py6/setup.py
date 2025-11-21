from setuptools import find_packages, setup
import os
import glob

package_name = 'pathing_py6'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all .srv files in the install
        (os.path.join('share', package_name, 'srv'), glob.glob('srv/*.srv')),
    ],
    install_requires=[
        'setuptools',
        'sensor_msgs',
        'trajectory_msgs',
        'geometry_msgs',
        'scipy',  # <-- Make sure 'scipy' is here
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='kkk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node1 = pathing_py6.node1:main', #add this
            'node2 = pathing_py6.node2:main', #add this
            'node3 = pathing_py6.node3:main', #add this
            'node4 = pathing_py6.node4:main', #add this
            'node5 = pathing_py6.node5_JointSpaceArbiter:main', #add this
            'node5_JointSpaceArbiter = pathing_py6.node5_JointSpaceArbiter:main', #add this
            'cutPizza = pathing_py6.cutPizza:main', #add this
        ],
    },
)
