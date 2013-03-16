AC_PREREQ([2.68])
AC_INIT([anrmalloc], [1.0])

AM_INIT_AUTOMAKE(subdir-objects foreign)
LT_INIT

# gmalloc enabled by default
AC_ARG_ENABLE([gmalloc], [--enable-gmalloc build libgmalloc], enable_gmalloc="$enableval", enable_gmalloc="yes")
AM_CONDITIONAL(BUILD_GMALLOC, test "$enable_gmalloc" = "yes")

# Checks for programs.
AC_PROG_CC
AC_PROG_CC_C_O
AM_PROG_CC_C_O
AC_PROG_AWK
AC_CHECK_TOOL(OBJCOPY, objcopy, no)
if test "${OBJCOPY}" = "no" ; then
    AC_MSG_ERROR(Cannot find objcopy.)
fi
LT_PATH_NM

# Checks for libraries.
AC_CHECK_LIB([pthread], [pthread_create])

PKG_CHECK_MODULES([membroker], [membroker], [have_membroker=yes], [have_membroker=no])
AS_IF([test "$have_membroker" = "yes"], [AC_DEFINE(HAVE_MEMBROKER, 1)])

# Architecture checks
AC_COMPILE_IFELSE([
    AC_LANG_PROGRAM([], [
        double a;
        __asm__ ("clz %0,%1" : "=r"(a) : "r"(a));
        ])],
    AC_DEFINE(CONFIG_USE_ARM_CLZ, yes)
])

AC_COMPILE_IFELSE([
    AC_LANG_PROGRAM([], [
        double a;
          __asm__ ("ldrd %0,[%1]" : "=r"(a) : "r"(&a));
        ])],
    AC_DEFINE(CONFIG_ALIGN_DOUBLE, yes)
])

AC_COMPILE_IFELSE([
    AC_LANG_SOURCE([
        #ifndef __CPU_HAS_ARM_POPCNT__
         #error - no popcnt
        #endif
        ])],
    AC_DEFINE(CONFIG_USE_ARM_POPCNT, yes)
])

AC_CONFIG_FILES(Makefile anrmalloc.pc gmalloc.pc)
AC_OUTPUT
AC_MSG_RESULT([
    Membroker:      $have_membroker
    gmalloc:        $enable_gmalloc
])
