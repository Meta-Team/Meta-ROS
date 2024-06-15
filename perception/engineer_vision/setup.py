from setuptools import find_packages, setup

package_name = 'engineer_vision'

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
    maintainer='robomaster',
    maintainer_email='kankefengjing@gmail.com',
    description='Image processing using YOLO',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obj_detection = engineer_vision.obj_detection:main',
            'img_saver = engineer_vision.img_saver:main'
        ],
    },
)
