from setuptools import setup
from glob import glob

package_name = 'control_services_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # индекс пакета
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # ВСЕ launch-файлы этого пакета
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='andrewa.work.il@gmail.com',
    description='Health publisher + ping (Trigger) service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_node = control_services_py.health_node:main',
        ],
    },
)
