# $FreeBSD$
#
# Common rules for building firmware.  Note this gets auto-included
# by the subdir Makefile's as a consequence of included bsd.kmod.mk.

_NAME=		iwlwifi-${NAME}-${VERSION}.ucode

IMG=		${_NAME}
KMOD=		${_NAME}

CLEANFILES+=	${IMG}

FIRMWS=		${IMG}:${IMG}:${VERSION}

#
# Note that a license ack is not needed for iwn/iwm/iwl.
#
#FIRMWARE_LICENSE=

${IMG}: ${SRCTOP}/sys/contrib/dev/iwlwifi/${IMG}
	cp ${.ALLSRC} ${.TARGET}

