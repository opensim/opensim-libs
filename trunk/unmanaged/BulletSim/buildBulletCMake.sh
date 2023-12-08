#! /bin/bash
# Script to build Bullet on a target system.

STARTDIR=$(pwd)

# The UNAME is either "Darwin" or otherwise. Note that env variable overrides
UNAME=${BULLETUNAME:-$(uname)}
# The MACH is either 'x86_64', 'aarch64', or assumed generic. Note env variable overrides.
MACH=${BULLETMACH:-$(uname -m)}
# Note that this sets BULLETDIR unless there is an environment variable of the same name
BULLETDIR=${BULLETDIR:-bullet3}

BUILDDIR=bullet-build

cd "${BULLETDIR}"
mkdir -p "${BUILDDIR}"
cd "${BUILDDIR}"

echo "=== Building Bullet in dir ${BULLETDIR} for uname ${UNAME} and arch ${MACH} into ${BUILDDIR}"

if [[ "$UNAME" == "Darwin" ]] ; then
    echo "=== Running cmake for Darwin"
    cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=OFF \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_OSX_ARCHITECTURES="arm64; x86_64" \
                -DCMAKE_CXX_FLAGS="-arch arm64 -arch x86_64" \
                -DCMAKE_C_FLAGS="-arch arm64 -arch x86_64 -fPIC -O2" \
                -DCMAKE_EXE_LINKER_FLAGS="-arch arm64 -arch x86_64" \
                -DCMAKE_VERBOSE_MAKEFILE="on" \
                -DCMAKE_BUILD_TYPE=Release
elif [[ "$UNAME" =~ "MINGW64*" ]] ; then
    cmake .. -G "Visual Studio 17 2022" \
            -DBUILD_BULLET3=ON \
            -DBUILD_EXTRAS=ON \
                -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF \
                -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                -DBUILD_OBJ2SDF_EXTRA=OFF \
                -DBUILD_SERIALIZE_EXTRA=OFF \
                -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                -DBUILD_HACD_EXTRA=ON \
                -DBUILD_GIMPACTUTILS_EXTRA=OFF \
            -DBUILD_CPU_DEMOS=OFF \
            -DBUILD_BULLET2_DEMOS=OFF \
            -DBUILD_ENET=OFF \
            -DBUILD_PYBULLET=OFF \
            -DBUILD_UNIT_TESTS=OFF \
            -DBUILD_SHARED_LIBS=OFF \
            -DINSTALL_EXTRA_LIBS=ON \
            -DINSTALL_LIBS=ON \
            -DCMAKE_CXX_FLAGS="-fPIC" \
            -DCMAKE_BUILD_TYPE=Release
else
    if [[ "$MACH" == "x86_64" ]] 
    then
        echo "=== Running cmake for arch $MACH"
        cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=OFF \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_CXX_FLAGS="-fPIC" \
                -DCMAKE_BUILD_TYPE=Release
    elif [[ "$MACH" == "aarch64" ]] 
    then
        echo "=== Running cmake for arch $MACH"
        cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=OFF \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_OSX_ARCHITECTURES="arm64" \
                -DCMAKE_CXX_FLAGS="-arch arm64" \
                -DCMAKE_C_FLAGS="-arch arm64 -fPIC -O2" \
                -DCMAKE_EXE_LINKER_FLAGS="-arch arm64" \
                -DCMAKE_BUILD_TYPE=Release
    else
        echo "=== Running cmake for generic arch"
        cmake .. -G "Unix Makefiles" \
                -DBUILD_BULLET3=ON \
                -DBUILD_EXTRAS=ON \
                    -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF \
                    -DBUILD_BULLET_ROBOTICS_EXTRA=OFF \
                    -DBUILD_OBJ2SDF_EXTRA=OFF \
                    -DBUILD_SERIALIZE_EXTRA=OFF \
                    -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON \
                    -DBUILD_HACD_EXTRA=ON \
                    -DBUILD_GIMPACTUTILS_EXTRA=OFF \
                -DBUILD_CPU_DEMOS=OFF \
                -DBUILD_BULLET2_DEMOS=OFF \
                -DBUILD_ENET=OFF \
                -DBUILD_PYBULLET=OFF \
                -DBUILD_UNIT_TESTS=OFF \
                -DBUILD_SHARED_LIBS=OFF \
                -DINSTALL_EXTRA_LIBS=ON \
                -DINSTALL_LIBS=ON \
                -DCMAKE_CXX_FLAGS="-fPIC" \
                -DCMAKE_BUILD_TYPE=Release
    fi
fi

# DEBUG DEBUG
echo "=== $(pwd)"
ls -l
echo "=== END"
# END DEBUG DEBUG

if [[ -e Makefile ]] ; then
    echo "=== Building Makefile"
    make -j4
fi
if [[ -e "BULLET_PHYSICS.sln" ]] ; then
    echo "=== Building BULLET_PHYSICS.sln"
    dotnet build -c Release BULLET_PHYSICS.sln
fi

# make install

# As an alternative to installation, move the .a files in to a local directory
#    Good as it doesn't require admin privilages
echo "=== Cleaning out any existing lib and include directories"
cd "$STARTDIR"
rm -rf lib
rm -rf include

echo "=== Moving .a files into ../lib"
cd "$STARTDIR"
mkdir -p lib
for afile in $(find "${BULLETDIR}/${BUILDDIR}" -name *.a) ; do
    cp "$afile" lib
done

echo "=== Moving .h files into ../include"
cd "$STARTDIR"
mkdir -p include
cd "${BULLETDIR}/src"
for file in $(find . -name \*.h) ; do
    xxxx="${STARTDIR}/include/$(dirname $file)"
    mkdir -p "$xxxx"
    cp "$file" "$xxxx"
done

# Move Bullet's VERSION file into lib/ so BulletSim can reference it
echo "=== Moving Bullet's VERSION file into ../lib"
cd "$STARTDIR"
mkdir -p lib
cp "${BULLETDIR}/VERSION" lib/

echo "=== Moving .h files from Extras into ../include"
cd "$STARTDIR"
cd "${BULLETDIR}/Extras"
for file in $(find . -name \*.h) ; do
    xxxx="${STARTDIR}/include/$(dirname $file)"
    mkdir -p "$xxxx"
    cp "$file" "$xxxx"
done
echo "=== Moving .inl files from Extras into ../include"
for file in $(find . -name \*.inl) ; do
    xxxx="${STARTDIR}/include/$(dirname $file)"
    mkdir -p "$xxxx"
    cp "$file" "$xxxx"
done
