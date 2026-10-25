from glob import glob
from setuptools import setup

setup(name='color_target_nav', version='0.1.0', packages=['color_target_nav'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/color_target_nav']),
                  ('share/color_target_nav', ['package.xml']),
                  ('share/color_target_nav/launch', glob('launch/*.py')),
                  ('share/color_target_nav/config', glob('config/*')),
                  ('share/color_target_nav/models', glob('models/*')),
                  ('share/color_target_nav/worlds', glob('worlds/*'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['navigator = color_target_nav.node:main',
                                        'watchdog = color_target_nav.watchdog:main']})
