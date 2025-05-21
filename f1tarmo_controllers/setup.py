from setuptools import find_packages, setup

package_name = 'f1tarmo_controllers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nlitz88',
    maintainer_email='nlitz88@todo.todo',
    description='Package with a node to translate a commanded twist motion to wheel speed and steering angle commands the f1tarmo can execute. This effectively serves as a vehicle command interface for the f1tarmo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bicycle_controller_node = f1tarmo_controllers.bicycle_controller_node:main'
        ],
    },
)
