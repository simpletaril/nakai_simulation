from setuptools import find_packages, setup

package_name = 'object_detection'

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
    maintainer='roey',
    maintainer_email='roey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = object_detection.object_detection:main',
            'mapping_node = object_detection.mapping_node:main',
            'detect_corner_node = object_detection.detect_corners:main',
            'depth_camera_node = object_detection.depth_camera_node:main',
            'slime_detection_node = object_detection.slime_detection:main',
            'hole_detection_node = object_detection.hole_detection_node:main'
        ],
    },
)
