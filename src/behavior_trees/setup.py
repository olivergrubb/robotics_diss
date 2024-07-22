from setuptools import find_packages, setup

package_name = 'behavior_trees'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py-trees', 'py-trees-ros', 'py-trees-ros-interfaces', 'rclpy', 'numpy', 'math'],
    zip_safe=True,
    maintainer='ollie',
    maintainer_email='ollie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['test_behavior = behavior_trees.trees.test_tree:main', 'vacuum_planner = behavior_trees.trees.vacuum_planner:main'
        ],
    },
)
