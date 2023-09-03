AM_CFLAGS = $(PTHREAD_CFLAGS)

lib_LTLIBRARIES = libusbtuner.la

libusbtuner_la_SOURCES = \
	libusbtuner.c

libusbtuner_la_LDFLAGS = -avoid-version -module -shared

libusbtuner_la_LIBADD = @LIBDLSYM_LIBS@ @PTHREAD_LIBS@
