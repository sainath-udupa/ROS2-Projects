from setuptools import find_packages, setup
from glob import glob #make sure this line is there
import os #make sure this line is also there

package_name = 'janyu_lab3_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share',package_name),glob('launch/*')), #add this line and make sure you don't add `your_package_name` instead of just `package_name` after 'share'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'publisher=janyu_lab3_ros2.publisher:main',
	'subscriber=janyu_lab3_ros2.subscriber:main',
	'sample=janyu_lab3_ros2.sample:main',	#do not add your launch file name, else it wont be executable if you do so. just add a random name (`sample` here)
        ],
    },
)
