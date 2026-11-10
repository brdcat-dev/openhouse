# Release-only static dependencies for the Linux package: Ceres without SuiteSparse
# (GPL parts), and nothing to bundle in lh-libs but what the host can't provide.
# Source paths in the built libraries (assert and log messages) name the vcpkg checkout,
# which is under the builder's home: mapped to "vcpkg".
set(LH_NO_BUILD_PATHS "-ffile-prefix-map=${VCPKG_ROOT_DIR}/buildtrees=vcpkg")
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_BUILD_TYPE release)
set(VCPKG_C_FLAGS "-fPIC -ffunction-sections -fdata-sections ${LH_NO_BUILD_PATHS}")
set(VCPKG_CXX_FLAGS "-fPIC -ffunction-sections -fdata-sections ${LH_NO_BUILD_PATHS}")
