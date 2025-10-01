from setuptools import setup

package_name = 'noise_maker_pck'

setup(
    name=package_name,
    version='0.0.1',
    packages=['noise_maker'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Random noise publisher node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noise_maker_node = noise_maker.noise_maker:main',
        ],
    },
)