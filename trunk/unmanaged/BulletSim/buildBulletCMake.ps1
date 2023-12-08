# Script to build Bullet on a Windows system

$MACH="x64"
$BULLETDIR="bullet3"

$BUILDDIR="bullet-build"

cd $BULLETDIR
New-Item -ItemType Directory -Path $BUILDDIR -Force
cd $BUILDDIR

echo "=== Building Bullet in dir $BULLETDIR for arch $MACH into $BUILDDIR"

cmake -G "Visual Studio 17 2022" -A $MACH -DDOTNET_SDK=ON -DBUILD_BULLET3=ON -DBUILD_EXTRAS=ON -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF -DBUILD_BULLET_ROBOTICS_EXTRA=OFF -DBUILD_OBJ2SDF_EXTRA=OFF -DBUILD_SERIALIZE_EXTRA=OFF -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON -DBUILD_HACD_EXTRA=ON -DBUILD_GIMPACTUTILS_EXTRA=OFF -DBUILD_CPU_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_ENET=OFF -DBUILD_PYBULLET=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_SHARED_LIBS=OFF -DINSTALL_EXTRA_LIBS=ON -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release ..

msbuild -p:Configuration=Release BULLET_PHYSICS.sln

# Copy the .lib files into the target lib directory
Set-Location ..\..
New-Item -ItemType Directory -Path lib -Force
Write-Host "=== Copy .lib files into the lib dir"
Get-ChildItem -Path bullet3\bullet-build\* -Include *.lib -Recurse | ForEach-Object {
    # echo "----- copying $_ to lib"
    Copy-Item $_ -Destination lib -Force
}

# Copy the .h and .inl files into the target include directory
Write-Host "=== Copy .h files into the include dir"
Get-ChildItem -Path bullet3\src\* -Include *.h,*.inl -Recurse | ForEach-Object {
    $xxxx = $_.Fullname -replace ".*\\bullet3\\src\\","include\"
    $xxxxDir = Split-Path -parent $xxxx
    if ( -not (Test-Path "$xxxxDir" -PathType container)) {
        echo "----- creating directory $xxxxDir"
        New-Item -ItemType Directory -Path $xxxxDir -Force | Out-Null
    }
    echo "----- copying $_ to $xxxx"
    Copy-Item $_ -Destination $xxxx -Force
}

# Copy the .h files from Extras into the target include directory
Write-Host "=== Copy Extras .h files into the include dir"
Get-ChildItem -Path bullet3\Extras\* -Include *.h,*.inl -Recurse | ForEach-Object {
    $xxxx = $_.Fullname -replace ".*\\bullet3\\Extras\\","include\"
    $xxxxDir = Split-Path -parent $xxxx
    if ( -not (Test-Path "$xxxxDir" -PathType container)) {
        echo "----- creating Extras directory $xxxxDir"
        New-Item -ItemType Directory -Path $xxxxDir -Force | Out-Null
    }
    echo "----- copying Extras $_ to $xxxx"
    Copy-Item $_ -Destination $xxxx -Force
}

# Copy Bullet's VERSION file into lib/ so BulletSim can reference it
Copy-Item -Path bullet3\VERSION -Destination lib

