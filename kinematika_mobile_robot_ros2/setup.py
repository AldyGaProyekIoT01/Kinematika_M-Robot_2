from setuptools import find_packages, setup

package_name = 'kinematika_mobile_robot'

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
    maintainer='student',
    maintainer_email='student@example.com',
    description='Forward and inverse kinematics nodes for a differential drive robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'forward_kinematics = kinematika_mobile_robot.forward_kinematics:main',
            'inverse_kinematics = kinematika_mobile_robot.inverse_kinematics:main',
        ],
    },
)
