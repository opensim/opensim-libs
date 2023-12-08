#! /bin/bash
# Script to build BulletSim Linux binaries.
# This presumes the bins and includes for Bullet are in BLIBDIR and BINCLUDEDIR

BASE=$(pwd)

BLIBDIR=${BLIBDIR:-./lib}
BINCLUDEDIR=${BINCLUDEDIR:-./include}

# Output file is ${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-${ARCH}.so
TARGETBASE=${TARGETBASE:-libBulletSim}

# CC=gcc
# CC=/usr/bin/g++
CC=/usr/bin/c++
# LD=/usr/bin/g++
LD=/usr/bin/c++

UNAME=${UNAME:-$(uname)}
ARCH=${ARCH:-$(uname -m)}

# Version of the Bullet engine that is being statically linked
BULLETVERSION=$(cat "${BLIBDIR}/VERSION")
# Version of the BulletSim glue that is being built and included
BULLETSIMVERSION=$(cat "VERSION")

BUILDDATE=$(date "+%Y%m%d")

# Kludge for building libBulletSim.so with different library dependencies
#    As of 20130424, 64bit Ubuntu needs to wrap memcpy so it doesn't pull in glibc 2.14.
#    The wrap is not needed on Ubuntu 32bit and, in fact, causes crashes.
if [[ "$ARCH" == "x86_64" ]] ; then
    WRAPMEMCPY=-Wl,--wrap=memcpy
else
    WRAPMEMCPY=
fi

# Pass version information into compilations as C++ variables
VERSIONCFLAGS="-D BULLETVERSION=$BULLETVERSION -D BULLETSIMVERSION=$BULLETSIMVERSION"
case $UNAME in
    "Linux")
        TARGET=${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-${ARCH}.so
        CFLAGS="-I${BINCLUDEDIR} -fPIC -g -fpermissive ${VERSIONCFLAGS}"
        LFLAGS="${WRAPMEMCPY} -shared -Wl,-soname,${TARGET} -o ${TARGET}"
        ;;
    "Darwin")
        CC=gcc
        LD=g++
        TARGET=${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-universal.dylib
        CFLAGS="-arch arm64 -arch x86_64 -O3 -I${BINCLUDEDIR} -g ${VERSIONCFLAGS}"
        LFLAGS="-v -dynamiclib -arch arm64 -arch x86_64 -o ${TARGET}"
        ;;
    *)
        TARGET=${TARGETBASE}-${BULLETVERSION}-${BUILDDATE}-${ARCH}.so
        CFLAGS="-I${IDIR} -fPIC -g -fpermissive ${VERSIONCFLAGS}"
        LFLAGS="${WRAPMEMCPY} -shared -Wl,-soname,${TARGET} -o ${TARGET}"
        ;;
esac

# All of the Bullet bin files
# BULLETLIBS=$(ls ${BLIBDIR}/*.a)
BULLETLIBS="${BLIBDIR}/libBulletDynamics.a ${BLIBDIR}/libBulletCollision.a ${BLIBDIR}/libLinearMath.a ${BLIBDIR}/libHACD.a"

# Just build everything
echo "=== Building target $TARGET from BulletSim glue ${BULLETSIMVERSION} and Bullet ${BULLETVERSION}"
${CC} ${CFLAGS} -c API2.cpp
${CC} ${CFLAGS} -c BulletSim.cpp
${LD} ${LFLAGS} API2.o BulletSim.o ${BULLETLIBS}
