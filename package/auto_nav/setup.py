from setuptools import find_packages, setup

package_name = 'auto_nav'

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
    maintainer='racer',
    maintainer_email='bavansp@gmail.com',
    entry_points={
        'console_scripts': [
        'revised_rpm=auto_nav.revised_rpm:main',
        'enco_conv=auto_nav.enco_conv:main',
        'end.py=auto_nav.end:main',
        'diff_motor_controller=auto_nav.diff_motor_controller_2:main',
        'odom_baselink=auto_nav.odom_baselink:main',
        ],
    },
)
