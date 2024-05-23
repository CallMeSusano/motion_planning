from setuptools import find_packages, setup

package_name = 'motion_plan'

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
    maintainer='YourName',
    maintainer_email='you@email.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = motion_plan.publisher_member_function:main',
                'listener = motion_plan.subscriber_member_function:main',
                'control = motion_plan.control:main',
                'measure = motion_plan.measure:main',
                'measure_2 = motion_plan.measure_2:main',
                #use this ones below
                'MeasureWheelRotation = motion_plan.MeasureWheelRotation:main',
                'MeasureWheelMap = motion_plan.MeasureWheelMap:main',
                'MeasureAngular = motion_plan.MeasureAngular:main',
        ],
    },
)
