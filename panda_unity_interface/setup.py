from setuptools import find_packages, setup

package_name = 'panda_unity_interface'

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
    maintainer='etfrobot',
    maintainer_email='etfrobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'moveit_controller = panda_unity_interface.moveit_controller:main',
                    'dummy_mission = panda_unity_interface.dummy_mission:main',
                    'test_controller = panda_unity_interface.test_cotroller:main',
                    'test_controller = panda_unity_interface.test_controller:main',

        ],
    },
)
