from setuptools import setup
import os
from glob import glob

package_name = 'jupiter_juno'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, package_name), 
            glob(package_name + '/*.dat')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jupiter Juno Team',
    maintainer_email='team@jupiterjuno.com',
    description='Jupiter Juno Driver Drowsiness Detection System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eye_detector_node = jupiter_juno.eye_detector_node:main',
            'alert_system_node = jupiter_juno.alert_system_node:main',
            'tts_node = jupiter_juno.tts_node:main',
            'speech_recognition_node = jupiter_juno.speech_recognition_node:main',
            'gemini_conversation_node = jupiter_juno.gemini_conversation_node:main',
        ],
    },
) 