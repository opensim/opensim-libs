#! /bin/bash
# Script to fetch the Bullet Physics Engine sources, build same
#    using the 'buildBulletSMake.sh' script, and then build BulletSim
#    .so's using 'buildBulletSim.sh'.
# This captures the steps needed and will be replaced by better scripts
#    and Github actions.
#
# This can build two versions of Bullet: one of current version and another
#    of Bullet version 2.86 which is the version of Bullet that was
#    used in the BulletSim binaries distributed with OpenSimulator
#    from 2015 to 2022.
# This also applies the BulletSim patches to the Bullet sources.

# Set these values to 'yes' or 'no' to enable/disable fetching and building
FETCHBULLETSOURCES=${FETCHBULLETSOURCES:-no}
BUILDBULLET2=${BUILDBULLET2:-no}    # usually don't need the old version
BUILDBULLET3=${BUILDBULLET3:-yes}

# Note that Bullet3 sources are build in "bullet3/" and the
#     these are copied into "bullet2/" and checkouted to the version 2 sources.

BASE=$(pwd)

if [[ "$FETCHBULLETSOURCES" == "yes" ]] ; then
    cd "$BASE"
    rm -rf bullet3

    echo "=== Fetching Bullet Physics Engine sources into bullet3/"
    git clone https://github.com/bulletphysics/bullet3.git

    if [[ "$BUILDBULLET2" == "yes" ]] ; then
        cd "$BASE"
        echo "=== Creating bullet2/ of Bullet version 2.86"
        rm -rf bullet2
        cp -r bullet3 bullet2
        cd bullet2
        git checkout tags/2.86 -b tag-2.86
    fi

    echo "=== Applying BulletSim patches to bullet3"
    cd "$BASE"
    cd bullet3
    for file in ../000* ; do cat $file | patch -p1 ; done

    if [[ "$BUILDBULLET2" == "yes" ]] ; then
        echo "=== Applying BulletSim patches to bullet2"
        cd "$BASE"
        cd bullet2
        for file in ../2.86-00* ; do cat $file | patch -p1 ; done
    fi
fi

cd "$BASE"

echo "=== Setting environment variables"
export BuildDate=$(date +%Y%m%d)
export BulletSimVersion=$(cat VERSION)
export BulletSimGitVersion=$(git rev-parse HEAD)
export BulletSimGitVersionShort=$(git rev-parse --short HEAD)
cd bullet3
export BulletVersion=$(cat VERSION)
export BulletGitVersion=$(git rev-parse HEAD)
export BulletGitVersionShort=$(git rev-parse --short HEAD)

echo "=== Creating version information file"
cd "$BASE"
rm BulletSimVersionInfo
touch BulletSimVersionInfo
echo "BuildDate=$BuildDate" > BulletSimVersionInfo
echo "BulletSimVersion=$BulletSimVersion" >> BulletSimVersionInfo
echo "BulletSimGitVersion=$BulletSimGitVersion" >> BulletSimVersionInfo
echo "BulletSimGitVersionShort=$BulletSimGitVersionShort" >> BulletSimVersionInfo
echo "BulletVersion=$BulletVersion" >> BulletSimVersionInfo
echo "BulletGitVersion=$BulletGitVersion" >> BulletSimVersionInfo
echo "BulletGitVersionShort=$BulletGitVersionShort" >> BulletSimVersionInfo
cat BulletSimVersionInfo

echo "=== removing libBulletSim-*"
rm libBulletSim-*.so

if [[ "$BUILDBULLET2" == "yes" ]] ; then
    echo "=== building bullet2"
    cd "$BASE"
    # Build the Bullet physics engine
    BULLETDIR=bullet2 ./buildBulletCMake.sh
    # Build the BulletSim glue/wrapper statically linked to Bullet
    ./buildBulletSim.sh
fi

if [[ "$BUILDBULLET3" == "yes" ]] ; then
    echo "=== building bullet3"
    cd "$BASE"
    # Build the Bullet physics engine
    BULLETDIR=bullet3 ./buildBulletCMake.sh
    # Build the BulletSim glue/wrapper statically linked to Bullet
    ./buildBulletSim.sh
fi
