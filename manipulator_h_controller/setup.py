from setuptools import find_packages, setup

package_name = 'manipulator_h_controller'

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
    maintainer='root',
    maintainer_email='gabriel.oliveira.dse@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_executor = manipulator_h_controller.pose_executor:main',
            'servo_control_node = manipulator_h_controller.servo_control_node:main', 
        ],
    },
)
