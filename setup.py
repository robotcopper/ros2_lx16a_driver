from setuptools import setup

package_name = 'ros2_lx16a_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='florisjousselin',
    maintainer_email='95591405+robotcopper@users.noreply.github.com',
    description='Package ROS2 for controlling LewanSoul LX-16A servos',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lx16a_node = ros2_lx16a_driver.lx16a_node:main',
        ],
    },
)
