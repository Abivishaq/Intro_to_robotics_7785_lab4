from setuptools import find_packages, setup

package_name = 'team7_lab4'

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
    maintainer='leonardo',
    maintainer_email='leonardo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goToPos = team7_lab4.goToPos:main',
            'chase_goal = team7_lab4.chase_object:main',
            'path_executor = team7_lab4.path_executor:main',
            'path_test_bed = team7_lab4.path_test_bed:main',
            'corner_pointer = team7_lab4.corner_pointer:main',
            'path_planner = team7_lab4.path_planner:main',
        ],
    },
)
