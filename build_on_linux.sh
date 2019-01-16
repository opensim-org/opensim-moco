#/bin/bash
git submodule update --init
mkdir ../moco_dependencies_build
cd ../moco_dependencies_build
cmake ../moco/dependencies \
    -DCMAKE_BUILD_TYPE=Release \
    -DOPENSIM_JAVA_WRAPPING=on \
    -DOPENSIM_PYTHON_WRAPPING=on \
    -DOPENSIM_PYTHON_VERSION=3
make --jobs 8
mkdir ../moco_build
cd ../moco_build
cmake ../moco \
    -DMOCO_JAVA_BINDINGS=on \
    -DMOCO_PYTHON_BINDINGS=on \
    -DOPENSIM_PYTHON_VERSION=3
make --jobs 8
