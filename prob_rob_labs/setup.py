from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'prob_rob_labs'

launch_files = glob('launch/*_launch.py') + \
    glob('launch/*_launch.xml') + \
    glob('launch/*.yaml') + \
    glob('launch/*.yml')

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'launch'), launch_files),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ilija Hadzic',
    maintainer_email='ih2435@columbia.edu',
    description='Prob Rob Labs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab_6_assignment_4 = lab_6_assignment_4.lab_6_assignment_4:main',
            'lab_6_assignment_2 = lab_6_assignment_2.lab_6_assignment_2:main',
            'final_project = final_project.final_project:main',
            'Lab_6_assignment_2 = Lab_6_assignment_2.Lab_6_assignment_2:main',
            'lab_6_assignment_1 = lab_6_assignment_1.lab_6_assignment_1:main',
            'prob_rob_labs = prob_rob_labs.prob_rob_labs:main',
            'lab_5_assignment_4_analyze = lab_5_assignment_4_analyze.lab_5_assignment_4_analyze:main',
            'lab_5_assignment_4_log = lab_5_assignment_4_log.lab_5_assignment_4_log:main',
            'lab_5_assignment_3 = lab_5_assignment_3.lab_5_assignment_3:main',
            'lab_5_assignment_2 = lab_5_assignment_2.lab_5_assignment_2:main',
            'lab_5_assignment_1 = lab_5_assignment_1.lab_5_assignment_1:main',
            'lab_4_assignment_8 = lab_4_assignment_8.lab_4_assignment_8:main',
            'lab_4_assignment_6_openloop = lab_4_assignment_6_openloop.lab_4_assignment_6_openloop:main',
            'lab_4_assignment_6 = lab_4_assignment_6.lab_4_assignment_6:main',
            'lab_4_assignment_3 = lab_4_assignment_3.lab_4_assignment_3:main',
            'lab_4_assignment_2 = lab_4_assignment_2.lab_4_assignment_2:main',
            'lab_4_assignment_1 = lab_4_assignment_1.lab_4_assignment_1:main',
            'lab_3_assignment_8 = lab_3_assignment_8.lab_3_assignment_8:main',
            'lab_3_assignment_6 = lab_3_assignment_6.lab_3_assignment_6:main',
            'lab_3_assignment_5 = lab_3_assignment_5.lab_3_assignment_5:main',
            'lab_2_question_4 = lab_2_question_4.lab_2_question_4:main',
            'lab_2_question_3 = lab_2_question_3.lab_2_question_3:main',
            'image_mean_feature_x = image_mean_feature_x.image_mean_feature_x:main',
            'flaky_door_opener = flaky_door_opener.flaky_door_opener:main',
        ],
    }
)
