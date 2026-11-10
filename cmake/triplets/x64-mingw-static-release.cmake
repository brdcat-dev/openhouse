# Release-only static MinGW dependencies for the Windows package. Per-function
# sections let the final link drop the unused parts of Ceres/glog/hidapi.
# Source paths in the built libraries (assert and log messages) name the vcpkg checkout,
# which is under the builder's home: mapped to "vcpkg".
set(LH_NO_BUILD_PATHS "-ffile-prefix-map=${VCPKG_ROOT_DIR}/buildtrees=vcpkg")
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_ENV_PASSTHROUGH_UNTRACKED PATH)
set(VCPKG_CMAKE_SYSTEM_NAME MinGW)
set(VCPKG_BUILD_TYPE release)
set(VCPKG_C_FLAGS "-ffunction-sections -fdata-sections ${LH_NO_BUILD_PATHS}")
set(VCPKG_CXX_FLAGS "-ffunction-sections -fdata-sections ${LH_NO_BUILD_PATHS}")
# Ceres's port replaces CMAKE_CXX_FLAGS on MinGW (-Wa,-mbig-obj), dropping the flags
# above; this later definition keeps its big-object flag and adds ours back.
if(PORT STREQUAL "ceres")
  set(VCPKG_CMAKE_CONFIGURE_OPTIONS "-DCMAKE_CXX_FLAGS=-Wa,-mbig-obj ${VCPKG_CXX_FLAGS}")
endif()
