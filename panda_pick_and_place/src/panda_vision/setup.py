from setuptools import find_packages, setup

package_name = 'panda_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuto',
    maintainer_email='1762934741@qq.com',
    description='Color detection package for Panda robot vision (RGB object detection)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = panda_vision.color_detector:main'
        ],
    },
)
