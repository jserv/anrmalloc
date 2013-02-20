#!/bin/sh

export GMALLOC_CFG=$srcdir/src/tests/test_gmalloc.cfg
exec ./test_gmalloc
