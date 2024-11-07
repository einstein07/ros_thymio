from setuptools import find_packages, setup

package_name = 'ros_thymio'

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
    maintainer='sindiso',
    maintainer_email='mkhsin035@myuct.ac.za',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'thymio_node = ros_thymio.vicon_subscriber_node:main'
        ],
    },
)
