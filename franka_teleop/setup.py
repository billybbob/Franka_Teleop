from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'franka_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ajoutez ce chemin pour les fichiers de lancement si nécessaire
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pinocchio',
    ],
    zip_safe=True,
    maintainer='vincent',
    maintainer_email='vincent@todo.todo',
    description='Téléopération d\'un robot Franka Fr3 en utilisant Pinocchio pour la cinématique inverse',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'franka_ik_solver = franka_teleop.franka_ik_solver:main',
            'franka_ig_solver = franka_teleop.franka_ig_solver:main',
            'mgd = franka_teleop.mgd:main',
            'switch_mode = franka_teleop.switch_mode:main',
            'fusion_joint_state = franka_teleop.fusion_joint_state:main',
            'guide_virtuel = franka_teleop.guide_virtuel:main',
            'force_joystick = franka_teleop.force_joystick:main',
            'fiole_pose_monitor = franka_teleop.fiole_pose_monitor:main',
            'distance_parois = franka_teleop.distance_parois:main',
        ],
    },
)
