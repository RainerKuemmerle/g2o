@echo off

setlocal

rem ----------------------------------
rem Locate vcpkg using environment variables falling back to sensible defaults
rem ----------------------------------
set "VcPkgDir=%USERPROFILE%\.vcpkg\vcpkg"
set "VcPkgTriplet=x64-windows"
if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" set "VcPkgDir=%VCPKG_ROOT_DIR%"
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" set "VcPkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
    
if not exist %VcPkgDir%\ (
echo Cannot find vcpkg directory %VcPkgDir%
echo Set VCPKG_ROOT_DIR to override
exit /b
)

pushd %VcPkgDir%

rem ==============================
rem Upgrade and Install packages.
rem ==============================
set "VcPkgLibs=eigen3 lapack-reference openblas suitesparse ceres"

echo vcpkg found at %VcPkgDir%...
echo installing %VcPkgLibs% for triplet %VcPkgTriplet%...

call %VcPkgDir%\vcpkg.exe upgrade %VcPkgLibs% --no-dry-run --triplet %VcPkgTriplet%
call %VcPkgDir%\vcpkg.exe install %VcPkgLibs% --triplet %VcPkgTriplet%

popd

endlocal & set "VcPkgDir=%VcPkgDir%" & set "VcPkgTriplet=%VcPkgTriplet%"
