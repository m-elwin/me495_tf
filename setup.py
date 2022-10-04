from setuptools import setup

package_name = 'me495_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
         ['package.xml', 'config/view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Elwin',
    maintainer_email='elwin@northwestern.edu',
    description='Example code for tf2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'in_out = me495_tf.in_out:in_out_entry',
            'tracker = me495_tf.tracker:tracker_entry'
        ],
    },
)
