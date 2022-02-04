from setuptools import setup

package_name = 'pathnode_vis'

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
    maintainer='y-okumura',
    maintainer_email='y-okumura@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathnode_vis = pathnode_vis.pathnode_vis:main',
            'latency_viewer = pathnode_vis.latency_viewer:main',
            'pubinfo_traverse = pathnode_vis.pubinfo_traverse:main',
            'parse_pub_info = pathnode_vis.parse_pub_info:main',
        ],
    },
)
