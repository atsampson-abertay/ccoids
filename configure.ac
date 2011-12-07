AC_PREREQ(2.59)
AC_INIT(ccoids.cc, 0.1, ats@offog.org)
AM_INIT_AUTOMAKE([foreign])

AC_PROG_CC
AC_PROG_CXX

PKG_CHECK_MODULES([CCOIDS], [ccsp-1.6 sdl])
CPPFLAGS="$CCOIDS_CFLAGS $CPPFLAGS"
LIBS="$CCOIDS_LIBS $LIBS"

AC_CHECK_LIB([SDL_gfx], [lineColor], ,
             [AC_MSG_ERROR(SDL_gfx is required)])
AC_CHECK_LIB([portmidi], [Pm_Initialize])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
