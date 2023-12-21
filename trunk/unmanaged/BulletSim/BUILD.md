# BUILDING THINGS

The [Bullet physics engine](https://github.com/bulletphysics/bullet3) is
available for OpenSimulator using the BulletSim plugin. This functionality
is provided by several pre-built binary executables for various target
architectures. These executables include DLL's, SO's, and DYNLIB's.

BulletSim consists of the C# code that is included in the OpenSimulator
sources, this C++ "glue" code which provides the interface between the C# code
and the Bullet physics engine, and the Bullet physics engine itself.

The steps are to fetch the Bullet physics engine sources, build it, then
build the BulletSim C++ glue code and staticlly link it with the built
Bullet physics engine.

Since Bullet is supplied as a binary, there are separate versions built
for different target operating systems and machine architectures. Thus
the built binary filename includes the version of Bullet used, the build
date, and the target machine architecture. Expect to see filenames like:

- `libBulletSim-3.25-20230122-x86_64.so` (Intel arch, Linux)
- `libBulletSim-3.25-20230122-aarch64.so` (ARM 64 bit arch, Linux)
- `libBulletSim-3.25-20230122-universal.dylib` (x86_64 and arm64 "universal binary", macOS)
- `libBulletSim-3.25-20230122-x86.64.dll` (Intel arch, Windows)

The selection of which binary to use must be configured in OpenSimulator.
This either requires copying the correct file to as default name or
editing a `.config` file.

# NOTES

- This builds with Bullet physics engine version 3+. Before 2023, 
  `BulletSim.dll` was built with Bullet version 2.86. The 3.25 version
  of Bullet has been tested and does not seem to make any
  difference to OpenSimulator operation as most of the Bullet changes
  have to do with APIs and integration with Python (thus PyBullet).
  Refer to some of the `.sh` files for the process of building the
  BulletSim binary with the previous version of Bullet.

- Only 64 bit architectures are supported.

# BUILDING

This "official" build scripts are called from `.github/workflows/build-dotnet6.yml`
which builds all the pieces using the Github action system.

For Linux, the script `makeBullets.sh` has the following build steps captured
in one script.

To build by hand, several scripts that are used in `build.yml`
are available. This builds the latest version of BulletSim with the latest
version of the Bullet physics engine.

1) Fetch the latest version from GitHub: https://github.com/bulletphysics/bullet3.

```
    cd trunk/unmanaged/BulletSim
    git clone --depth 1 --single-branch https://github.com/bulletphysics/bullet3.git
```

2) Apply all the patches for bullet:

```
	cd bullet3 ; for file in ../*.patch ; do cat $file | patch -p1 ; done
```

There are some small changes that are needed for the using Bullet for
distributed physics (physics simulation in both the serve and the client).

There are separate patch files for Bullet version 2.86. These all start
with the string "2.86-". Refer to `makeBullets.sh` for an example of pulling
and patching the 2.86 version of Bullet.

3) Build the Bullet physics engine

  a) Windows:

    Install CMake for Windows.

    `buildBulletCMake.ps1` builds the Bullet physics engine. Note that this
    is a PowerShell script.

    If one can not install `CMake` but has Visual Studio, `buildBulletVS.bat` will
    build `bullet3/build3/vs2010/0Bullet3Solution.sln` which can be used by 
    any modern Visual Studio to build the Bullet physics engine.
    Note that after this step, the Bullet physics engine binaries will be in
    the `lib` directory and the include files must be copied into `include/`.

  b) Linux and macOS:
    
    The script "buildBulletCMake.sh" has the appropriate cmake and compilation
    commands for Linux and IOS.
    The script builds bullet static libraries and copies them into local directories.

```
    ./buildBulletCMake.sh
```

4) Build BulletSim

  a) Windows:

    Generate version file information:

```
    bash buildVersionInfo.sh
```

    Build BulletSim:

```
    .\buildBulletSim.ps1
```

  b) Linux and macOS:

    Run BulletSim compile and link script:

```
    ./buildBulletSim.sh
```

    This builds a file with a name like: `libBulletSim-3.25-20230111-x86_64.so`.
    Copy this file in to the OpenSimulator `bin/lib64` directory and edit
    `OpenSim.Region.PhysicsModule.BulletS.dll.config` to point to this file for
    the machine architecture you are running on.

