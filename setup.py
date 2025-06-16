from setuptools import find_packages, setup

package_name = 'line_follower_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/line_follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sai',
    maintainer_email='saikr2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',

	entry_points={
		'console_scripts': [
		    'line_detector = line_follower_drone.line_detector:main',
		    'controller = line_follower_drone.controller:main',
		    'keyboard_controller = line_follower_drone.keyboard_controller:main',
		],
	},
)
