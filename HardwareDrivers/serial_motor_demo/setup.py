from setuptools import setup
import os
from glob import glob

package_name = 'serial_motor_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/action', ['action/Enable.action']),
        (os.path.join('share', package_name, 'action'), glob(os.path.join('action', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='newans',
    maintainer_email='josh.newans@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = serial_motor_demo.gui:main',
            'driver = serial_motor_demo.driver:main',
            'pooping_action_server = serial_motor_demo.pooping_action_server:main'
        ],
    },
)
