#!/bin/sh
set -e
halcompile --install test_define1.comp
halcompile --install test_use1.comp
MSGD_OPTS="--stderr" halrun dotest.hal
