from setuptools import setup
import os
from glob import glob

package_name = 'CRAP_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinkai',
    maintainer_email='oliverjkqiu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'test_controller = CRAP_navigation.test_controller:main',
        'test_random = CRAP_navigation.test_random:main',
        'test_found = CRAP_navigation.test_found:main',
        'test_bucketAction = CRAP_navigation.test_bucketAction:main',
        'fake_ball_publisher = CRAP_navigation.fake_ball_publisher:main',
        'main_controller = CRAP_navigation.main_controller:main',
        'amclinit = CRAP_navigation.amclinit:main'
        # 'random_searchig_test = CRAP_navigation.random_searchig_test:main',
        ],
    },
)
