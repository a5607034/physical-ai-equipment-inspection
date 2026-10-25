from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project Contributor',
    maintainer_email='anonymous@example.com',
    description=(
        'ROS 2-based autonomous equipment inspection system '
        'using LLM planning, waypoint navigation, '
        'camera capture, and VLM inspection.'
    ),
    license='LicenseRef-Proprietary',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_planner = my_py_pkg.llm_planner:main',
            'waypoint_controller = my_py_pkg.waypoint_controller:main',
            'inspection_camera = my_py_pkg.inspection_camera:main',
            'inspection_vlm = my_py_pkg.inspection_vlm:main',
        ],
    },
)
