from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['web_dashboard'],
    package_dir={'': 'src'},	
    scripts=['scripts/web_dashboard']
)

setup(**setup_args)
