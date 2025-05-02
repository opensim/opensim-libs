This is a code repository containing various 3rd party library packages used by opensim (as version >= 0.9.3).


# Managed code libraries

In the case of managed code libraries, these will be dotNet .dll assemblies compiled to run on dotnet platform. These are placed in Opensim `bin/` folder

# Native code libraries

In the case of unmanaged (native) code, like C/C++ libs, most of these are placed in folder `bin/lib64`.
(Since 0.9.3.0 32bit is no longer supported, so folder bin/lib32 alternative place was removed).

Most opensim projects that depend on native code libraries will need proper load code.

Some may be based on DllmapConfigHelper to find the native libraries and need a corresponding `projectName.dll.config` file setup so the runtime can find and load the correct library for the runtime platform and cpu. see for example `bin/OpenMetaverse.dll.config` on opensim source.

A few cases may use other arrangement

# Nuget

Opensim will avoid to use Nuget whenever possible. In same cases we may extract and use only the files we need out of the nuget packages garbage

# License

Each library package has its own Open Source License included on its folder tree.