from setuptools import setup

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathan Sprague',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = ros2_aruco.aruco_node:main',
            'aruco_generate_marker = ros2_aruco.aruco_generate_marker:main' ,
            'aruco_node_explaination = ros2_aruco.aruco_node_explaination:main',
            'aruco_node_original = ros2_aruco.aruco_node_original:main', 
            'aruco_node_camera = ros2_aruco.aruco_node_camera:main'
        ],
    },
)
