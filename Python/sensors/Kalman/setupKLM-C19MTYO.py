# from distutils.core import setup
# from Cython.Build import cythonize
# import numpy
# setup(
#     name='HelloClass',
#     ext_modules=cythonize('helloClass.pyx'),
#     include_dirs=[numpy.get_include()]
# )

from distutils.core import setup
from Cython.Build import cythonize
import numpy
from distutils.extension import Extension


import Cython.Compiler.Options
Cython.Compiler.Options.annotate = True

# setup(name='HelloClass',
#       ext_modules=cythonize([Extension("helloClass", ["helloClass.pyx"], include_dirs=[numpy.get_include()])],
#                             annotate=True)
# )
setup(name='KalmanData',
      ext_modules=cythonize([Extension("KalmanData", ["KalmanData.pyx"], include_dirs=[numpy.get_include()])],
                            language_level=3,
                            annotate=True)
      )

#
setup(name='Klm_Estimator',
      ext_modules=cythonize([Extension("Klm_Estimator", ["Klm_Estimator.pyx"], include_dirs=[numpy.get_include()])],
                            language_level=3,
                            annotate=True)
)

# setup(
#     name='HelloClass',
#     ext_modules=cythonize([Extension("helloClass", ["helloClass.pyx"], include_dirs=[numpy.get_include()])],
#                           language_level=3,
#                           annotate=True),
#     # compiler_directives={'language_level': "3"}, cython_directives={"annotate": True},
# )
