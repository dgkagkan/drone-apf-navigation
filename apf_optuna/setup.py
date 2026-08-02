from setuptools import find_packages, setup


package_name = 'apf_optuna'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='dimitris-gkagkanakis',
    maintainer_email='jim.gaganakis@gmail.com',
    description='Optuna trial orchestration and telemetry evaluation for VTOL APF tuning.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'optimize = apf_optuna.optimizer:main',
            'seed_study = apf_optuna.seed_study:main',
        ],
    },
)
