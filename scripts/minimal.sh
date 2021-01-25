#!/usr/bin/env bash

# check if haros is installed
if python -c 'import pkgutil; exit(not pkgutil.find_loader("haros"))'; then
    echo 'Found HAROS in the system.'
else
    echo 'HAROS was not found. Please install it first.'
    exit 1
fi

# get the directory this script is in
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR/../projects

haros full -n -p minimal.yaml
