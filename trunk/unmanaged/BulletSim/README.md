# BulletSim Binary Build

The BulletSim physics module for [OpenSimulator](http:opensimulator.org)
is distributed as a binary file.
The filenames are different for the various architectures and operating
system types. Names are usually:

- `BulletSim*.dll` - Windows
- `libBulletSims*.so` - Linux
- `libBulletSim*.dynlib` - IOS

The names could have architecture info in them like `x86_64`, `aarch64`, or
`universal` (with the `universal` possibly containing the images for multiple
Apple architectures).

The binary file that is selected and run is specified in the OpenSimulator
file `bin/OpenSim.Region.PhysicsModule.BulletS.dll.config` which selects
the binary file to load based on the operating system.

See the file in this directory named `BUILD.md`
for instructions on how to build the binary files.
The build steps include downloading and building the Bullet physics engine binaries
and then building and linking in the BulletSim wrapper.
There are many files in this directory related to building BulletSim using
Github actions.

Development on BulletSim is also done in
[Misterblue's BulletSim repository](https://github.com/Misterblue/BulletSim)
which has Github actions to build BulletSim for the various target
architectures.

Development for IOS has been done by
Cuga-rajal so refer to the
[opensim-apple-arm64](https://github.com/cuga-rajal/opensim_apple_arm64) repository
for that part of the world.
Cuga built and signed a BulletSim release for OpenSimulator which can
be found in the released `bin/lib64` directory of OpenSimulator.
