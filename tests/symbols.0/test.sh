#!/bin/sh
set -xe
halcompile --install test_define.comp
halcompile --install test_use.comp
! MSGD_OPTS="--stderr" halrun dotest.hal
