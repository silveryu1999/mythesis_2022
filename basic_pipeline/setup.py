from setuptools import setup

package_name = 'basic_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='silveryu1999',
    maintainer_email='silveryu1999@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'camera = basic_pipeline.camera:main',
        	'scheduler = basic_pipeline.scheduler:main',
        	'detector = basic_pipeline.detector:main',
        	'networker = basic_pipeline.networker:main',
        	'networker_ros2 = basic_pipeline.networker_ros2:main',
        	'tracker = basic_pipeline.tracker:main',
        	'collector = basic_pipeline.collector:main',
        	'displayer = basic_pipeline.displayer:main',
        	'server_ros2 = basic_pipeline.server_ros2:main',
        	'monitor = basic_pipeline.monitor:main',
        ],
    },
)
