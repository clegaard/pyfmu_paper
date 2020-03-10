
from setuptools import setup, find_packages
setup(
    name="pyfmu_paper",
    version="0.0.1",
    packages=find_packages('src'),
    package_dir={'': 'src'},

    install_requires=["numpy", "autograd", "scipy"],


    #entry_points={'console_scripts': ['pyfmu=pybuilder.pyfmu:main']}


)
