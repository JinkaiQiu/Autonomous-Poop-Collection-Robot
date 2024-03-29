from setuptools import setup
from glob import glob

package_name = 'learning_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hu Chunxu',
    maintainer_email='huchunxu@guyuehome.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'follow_1pp      = learning_node.follow_1pp:main',
         'node_object            = learning_node.node_object:main',
         'node_object_webcam     = learning_node.node_object_webcam:main',
         'node_object_webcam_1pp     = learning_node.node_object_webcam_1pp:main',
         'node_object_1pp           = learning_node.node_object_1pp:main',
         'node_webcam_1pp_shape = learning_node.node_webcam_1pp_shape:main',
         'get_hsv = learning_node.get_hsv:main',
        ],
    },
)
