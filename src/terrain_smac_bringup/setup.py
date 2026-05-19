from setuptools import setup

package_name = 'terrain_smac_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/terrain_smac.launch.py']),
        ('share/' + package_name + '/config',
            ['config/smac_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ATALAICEEGG',
    maintainer_email='1825703941@qq.com',
    description='Nav2 SmacPlanner2D bringup for terrain navigation',
    license='MIT',
    tests_require=['ament_lint_auto', 'ament_lint_common'],
    entry_points={
        'console_scripts': [
            'smac_test_client = terrain_smac_bringup.smac_test_client:main',
        ],
    },
)
