#!/bin/bash -le
d=$(dirname $(readlink -f "$0"))

build_dir="$d/build"
do_clear=false
release="Debug"
rel="-DCMAKE_BUILD_TYPE="

function msg() {
    (>&2 echo $@)
}

for arg in "$@"; do
    if [[ "$arg" == "-c" ]]; then
        do_clear=true
    fi
    if [[ "$arg" == "-r" ]]; then
        release="Release"
    fi
done

if [[ ! -d "$build_dir" ]]; then
    msg "creating build directory"
    mkdir "$build_dir"
else
    if [[ "$do_clear" == true ]]; then
        >&2 read -p "remove $build_dir/* ? (y/n) " -n 1 -r
        echo >&2
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf --preserve-root --one-file-system -- "$build_dir/*"
        else
            msg "abort."
            exit 1
        fi
    fi
fi

msg "initializing in $build_dir"
cd "$build_dir"
cmake -DCMAKE_TOOLCHAIN_FILE="${d}/etc/cmake/armgcc.cmake" "${rel}${release}" ..
