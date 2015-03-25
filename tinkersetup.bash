#!/bin/bash
# tinkersetup.bash
# created by bss at 2014-12-28
# Last modified: 2015-03-21, 13:58:40

if [ -n $TINKER_WORKSPACE ]; then
    export TINKER_WORKSPACE=$(dirname $BASH_SOURCE)
fi

source $TINKER_WORKSPACE/dev/devel/setup.bash
source $TINKER_WORKSPACE/driver/devel/setup.bash --extend
source $TINKER_WORKSPACE/logic/devel/setup.bash --extend
source $TINKER_WORKSPACE/decision/devel/setup.bash --extend
source $TINKER_WORKSPACE/downloaded/devel/setup.bash --extend

