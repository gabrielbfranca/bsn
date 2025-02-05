from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
 
    # fetch values from package.xml
setup_args = generate_distutils_setup(
        packages=['component'],
        package_dir={'': 'src'},
        package_data={'component': ['sensor_module.so']})
   
setup(**setup_args)