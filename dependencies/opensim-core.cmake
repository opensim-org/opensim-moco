# This file is included by the CMakeLists.txt in this directory.

# We list (and update) the opensim-core submodule commit here so that AppVeyor
# will invalidate its cached opensim-core installation if we change the commit.
# This commented commit hash is not actually used in the superbuild.
# opensim-core commit:
# 2f5d4b09722e57359e2ff1787328527050a4d34e

AddDependency(NAME       opensim-core
              URL        ${CMAKE_SOURCE_DIR}/../opensim-core
              CMAKE_ARGS
                    -DBUILD_API_EXAMPLES:BOOL=OFF
                    -DBUILD_TESTING:BOOL=OFF
                    -DBUILD_JAVA_WRAPPING:BOOL=${OPENSIM_JAVA_WRAPPING}
                    -DBUILD_PYTHON_WRAPPING:BOOL=${OPENSIM_PYTHON_WRAPPING}
                    -DOPENSIM_PYTHON_VERSION:STRING=${OPENSIM_PYTHON_VERSION}
                    -DOPENSIM_DEPENDENCIES_DIR:PATH=${CMAKE_INSTALL_PREFIX}
                    -DOPENSIM_C3D_PARSER:STRING=ezc3d
                    -DOPENSIM_INSTALL_UNIX_FHS:BOOL=${OPENSIM_INSTALL_UNIX_FHS})


if(SUPERBUILD_opensim-core)

    # OpenSim's dependencies.
    AddDependency(NAME       ezc3d
                  DEFAULT    OFF
                  GIT_URL        https://github.com/pyomeca/ezc3d.git
                  GIT_TAG        Release_1.3.2
                  CMAKE_ARGS -DBUILD_EXAMPLE:BOOL=OFF)

    AddDependency(NAME simbody
                  GIT_URL    https://github.com/simbody/simbody.git
                  GIT_TAG    Simbody-3.7
                  CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF 
                             -DBUILD_TESTING:BOOL=OFF)

    AddDependency(NAME       docopt
                  GIT_URL    https://github.com/docopt/docopt.cpp.git
                  GIT_TAG    3dd23e3280f213bacefdf5fcb04857bf52e90917
                  CMAKE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING=_d)

    add_dependencies(opensim-core ezc3d simbody docopt)
endif()



