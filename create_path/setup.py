from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'create_path'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanawat',
    maintainer_email='thanawat@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	    "save_path = create_path.save_path:main",
            	"follow_path = create_path.follow_path:main",
                "show_ocgm = create_path.show_ocgm:main",
        ],
    },
)
