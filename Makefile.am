CLEANFILES =

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = anrmalloc.pc

lib_LTLIBRARIES = libanrmalloc.la
libanrmalloc_la_SOURCES = \
	src/anrcore.c \
	src/parse_config.c \
	src/anrmalloc.c \
	src/anrcore.h \
	src/anrmalloc.h \
	src/parse_config.h

noinst_LTLIBRARIES = libanrmalloc_withtests.la
libanrmalloc_withtests_la_SOURCES = $(libanrmalloc_la_SOURCES)
libanrmalloc_withtests_la_CFLAGS = -DBUILD_TESTS

# This is not portable
rununittests.c: libanrmalloc_withtests.la
	$(NM) .libs/libanrmalloc_withtests.a | awk '/__unit_test_/{ print "extern void " $$3 "();"}' > $@
	echo 'int main () {' >> $@
	$(NM) .libs/libanrmalloc_withtests.a | awk '/__unit_test_/{ print $$3 "();" }' >> $@
	echo 'return 0; }' >> $@
CLEANFILES += rununittests.c

pkginclude_HEADERS = src/anrmalloc.h
dist_sysconf_DATA = src/anrmalloc.cfg

check_PROGRAMS = rununittests
rununittests_LDADD = libanrmalloc_withtests.la
TESTS = rununittests

if BUILD_GMALLOC
pkgconfig_DATA += gmalloc.pc
lib_LTLIBRARIES += libgmalloc.la
libgmalloc_la_SOURCES = \
	src/anrcore.c \
	src/parse_config.c \
	src/gmalloc.c \
	src/anrcore.h \
	src/anrmalloc.h \
	src/parse_config.h \
	src/gmalloc.h \
	src/gmalloc.lnk
libgmalloc_la_CFLAGS = $(membroker_CFLAGS)
libgmalloc_la_LDFLAGS = \
	-Wl,--version-script=$(srcdir)/src/gmalloc.lnk \
	$(membroker_LIBS)
libgmalloc_la_DEPENDENCIES = src/gmalloc.lnk
pkginclude_HEADERS += src/gmalloc.h
dist_sysconf_DATA += src/gmalloc.cfg

check_PROGRAMS += test_gmalloc
test_gmalloc_SOURCES = src/tests/test_gmalloc.c
test_gmalloc_CFLAGS = -I$(srcdir)/src
test_gmalloc_LDADD = libgmalloc.la
dist_check_SCRIPTS = src/tests/run-test-gmalloc.sh

TESTS += src/tests/run-test-gmalloc.sh
EXTRA_DIST = src/tests/test_gmalloc.cfg
endif

dist_bin_SCRIPTS = src/tools/find-leaks.py
