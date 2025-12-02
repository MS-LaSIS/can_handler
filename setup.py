from setuptools import setup
import os
from glob import glob

package_name = 'can_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=[
        'setuptools',
        'python-can',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='CAN bus message handler',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_handler_node.py = can_handler.can_handler_node:main',
            'serial_can_handler_node.py = can_handler.serial_can_handler_node:main',
            'simulate_can.py = can_handler.simulate_can:main',
        ],
    },
)
