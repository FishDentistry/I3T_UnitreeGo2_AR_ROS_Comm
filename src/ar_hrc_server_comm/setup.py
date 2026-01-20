from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'ar_hrc_server_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsl',
    maintainer_email='cfronk12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsenseTopicsSubscriber = ar_hrc_server_comm.RealsenseTopicsSubscriber:main',
            'robotStateComm = ar_hrc_server_comm.RobotStateComm:main',
            'robotCommandListener = ar_hrc_server_comm.RobotCommandListener:main',
            'voxelMapper = ar_hrc_server_comm.CreateLidarVoxelMap:main',
            'stackedOccupancyGrid = ar_hrc_server_comm.StackedOccupancyGrid:main',
            'occupancyBlobFilter = ar_hrc_server_comm.OccupancyBlobFilter:main',
            'findArucoFrame = ar_hrc_server_comm.FindArucoFrame:main',
            'findAprilTagFrame_TCP = ar_hrc_server_comm.FindAprilTagFrame_TCP:main',
            'sendRobotPos_TCP = ar_hrc_server_comm.SendRobotPos_TCP:main',
            'showObjsRviz = ar_hrc_server_comm.ShowObjsRviz:main'
        ],
    },
)
