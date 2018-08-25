@echo off

setlocal

rem ----------------------------------
rem Locate vcpkg using environment variables falling back to sensible defaults
rem ----------------------------------
set "VcPkgDir=%USERPROFILE%\.vcpkg\vcpkg"
set "VcPkgTriplet=x64-windows"
if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" set "VcPkgDir=%VCPKG_ROOT_DIR%"
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" set "VcPkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
    
pushd %VcPkgDir%

rem ==============================
rem Upgrade and Install packages.
rem ==============================
set "VcPkgLibs=eigen3 suitesparse clapack openblas ceres"

echo vcpkg found at %VcPkgDir%...
echo installing %VcPkgLibs% for triplet %VcPkgTriplet%...

call vcpkg upgrade %VcPkgLibs% --no-dry-run --triplet %VcPkgTriplet%
call vcpkg install %VcPkgLibs% --triplet %VcPkgTriplet%

popd

endlocal & set "VcPkgDir=%VcPkgDir%" & set "VcPkgTriplet=%VcPkgTriplet%"
