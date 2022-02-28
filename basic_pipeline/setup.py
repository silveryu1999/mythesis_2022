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
        	'server = basic_pipeline.server:main',
        	'bs_client = basic_pipeline.client_member_function:main',
        ],
    },
)
