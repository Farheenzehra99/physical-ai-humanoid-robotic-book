from setuptools import setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Physical AI Team',
    maintainer_email='dev@physical-ai-robotics.dev',
    description='Simple ROS 2 example',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = hello_ros2.publisher:main',
        ],
    },
)
