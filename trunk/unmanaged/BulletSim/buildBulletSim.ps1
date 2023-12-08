
$BULLETVERSION = Get-Content -Path .\lib\VERSION
$BULLETSIMVERSION = Get-Content -Path .\VERSION

Set-Item -Path Env:CL -Value "/DBULLETVERSION#$BULLETVERSION /DBULLETSIMVERSION#$BULLETSIMVERSION"

msbuild -p:Configuration=Release BulletSim.sln
