import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'botdylan'

# Helper function to recursively collect files in subdirectories
def recursive_files(path):
    files = []
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            files.append(os.path.join(dirpath, f))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['botdylan', 'botdylan.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/**/*.dae', recursive=True)),  # Include meshes if applicable
        ('share/' + package_name + '/code', glob('code/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Bot Dylan Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory = botdylan.trajectory:main',
        ],
    },
)
