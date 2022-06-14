"""Visualization tool for TILDE."""
from setuptools import setup

package_name = 'tilde_vis'

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
            'tilde_vis = tilde_vis.tilde_vis:main',
            'latency_viewer = tilde_vis.latency_viewer:main',
            'message_tracking_tag_traverse = tilde_vis.message_tracking_tag_traverse:main',
            'parse_message_tracking_tag = tilde_vis.parse_message_tracking_tag:main',
        ],
    },
)
