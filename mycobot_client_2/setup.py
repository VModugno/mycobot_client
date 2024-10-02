import os
from glob import glob
from setuptools import setup

package_name = 'mycobot_client_2'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=package_files(data_files, ['models/', 'configs/', 'launch/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Valerio',
    maintainer_email='v.modugno@ucl.ac.uk',
    description='Control the Elephant Robotics MyCobot 280',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cobot_client = mycobot_client_2.cobot_client:main',
            'cobot_ik = mycobot_client_2.ik:main',
            'cobot_ik_demo = mycobot_client_2.ik_demo:main',
            'run_task_simple = mycobot_client_2.run_task_simple:main'
        ],
    },
)
