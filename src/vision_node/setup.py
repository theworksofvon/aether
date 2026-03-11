from setuptools import setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'vision_node = vision_node.vision_node:main'
        ],
    },
)