
from setuptools import find_packages
from setuptools import setup

setup(
      name="checkers_app",
      version ="1.0.0",
      author="ASE",
      packages=['utm_monte_carlo_simulation', 'scripts'],
      #packages = find_packages(),
      #scripts=[],
      install_requires=[],
      license="MIT",
      description="Monte Carlo Traffic Simulation for UTM"
      )