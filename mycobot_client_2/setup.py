from setuptools import setup


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valerio',
    maintainer_email='v.modugno@ucl.ac.uk',
    description='Control the Elephant Robotics MyCobot 280',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cobot_client = mycobot_interface_2.cobot_client:main'
        ],
    },
)