#! /bin/bash
# Script for building "BulletSimVersionInfoFile" which contains application and git
#    version information.
# Also sets this information into the environment
# This file exists as an alternative to the building of the version info file in
#    .github/workflows/build.yml

export BuildDate=$(date +%Y%m%d)

export BulletSimVersion=$(cat VERSION)
export BulletSimGitVersion=$(git rev-parse HEAD)
export BulletSimGitVersionShort=$(git rev-parse --short HEAD)
cd bullet3
export BulletVersion=$(cat VERSION)
export BulletGitVersion=$(git rev-parse HEAD)
export BulletGitVersionShort=$(git rev-parse --short HEAD)

cd ..
echo "BuildDate=$BuildDate" > BulletSimVersionInfo
echo "BulletSimVersion=$BulletSimVersion" >> BulletSimVersionInfo
echo "BulletSimGitVersion=$BulletSimGitVersion" >> BulletSimVersionInfo
echo "BulletSimGitVersionShort=$BulletSimGitVersionShort" >> BulletSimVersionInfo
echo "BulletVersion=$BulletVersion" >> BulletSimVersionInfo
echo "BulletGitVersion=$BulletGitVersion" >> BulletSimVersionInfo
echo "BulletGitVersionShort=$BulletGitVersionShort" >> BulletSimVersionInfo
