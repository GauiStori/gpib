#!/usr/bin/env python
import sys
if sys.version_info < (3,10):
    from distutils.core import setup,Extension
else:
    from setuptools import setup, Extension
setup(name="gpib",
	version="1.0",
	description="Linux GPIB Python Bindings",
	py_modules = ['Gpib'],
	ext_modules=[
		Extension("gpib",
		["gpibinter.c"],
		include_dirs=["../../include"],
		library_dirs = ['../../lib/.libs'],
		libraries=["gpib", "pthread"]
	)]
)
