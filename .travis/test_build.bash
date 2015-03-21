#!/bin/bash
# -*- coding: utf-8 -*-
# test_build.bash
# created by bss at 2015-03-21
#

make downloaded
make driver pkg="d_say d_cv"
make logic pkg=l_sphinx_wrapper
make decision pkg=answer_questions

exit 0

