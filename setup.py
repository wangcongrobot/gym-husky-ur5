from setuptools import setup

setup(
    name='gym-husky-ur5',
    version='0.0.1',
    description='A gym mujoco environment of husky ur5 and 3 finger gripper',
    author='Cong Wang',
    author_email='wangcongrobot@gmail.com',
    install_requires=['gym', 
                      'numpy',
                      'mujoco_py']
)