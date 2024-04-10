This is a code repository containing various 3rd party library packages used by opensim (as version >= 0.9.3).

Opensim does not use Nuget (and hopefuly never will)

OpenSim checks out with most of these libs precompiled in the bin folder.

In the case of managed code, these will be .dll assemblies compiled to run on dotnet platform.

In the case of unmanaged (native) code, like C/C++ libs with, most native libraries will be placed in folder bin/lib64.
Since 0.9.3.0 32bit is no longer supported, so folder bin/lib32 alternative place was removed.

Most opensim project will need proper load code and a corresponding projectName.dll.config file setup so the runtime can load the correct native library for the runtime platform and cpu

see for example `bin/OpenMetaverse.dll.config` on opensim source.

A few cases may use other arrangement

