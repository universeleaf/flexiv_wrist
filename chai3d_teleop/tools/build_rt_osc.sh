#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${project_root}/../.." && pwd)"
rdk_source="${workspace_root}/flexiv_rdk-1.9"
rdk_prefix="${workspace_root}/rdk-install"
rdk_build="${workspace_root}/build-flexiv-rdk-1.9"
demo_build="${project_root}/build_rt_osc"

if [[ ! -d "${rdk_source}" ]]; then
  echo "Missing official Flexiv RDK v1.9 source: ${rdk_source}" >&2
  exit 1
fi
if [[ ! -f "${rdk_prefix}/lib/cmake/RBDyn/RBDynConfig.cmake" ]]; then
  echo "RDK dependencies are not installed in ${rdk_prefix}." >&2
  echo "Run from ${rdk_source}:" >&2
  echo "  bash thirdparty/build_and_install_dependencies.sh ${rdk_prefix} 8" >&2
  exit 1
fi

cmake -S "${rdk_source}" -B "${rdk_build}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${rdk_prefix}" \
  -DCMAKE_INSTALL_PREFIX="${rdk_prefix}"
cmake --build "${rdk_build}" --target install -j 8

cmake -S "${project_root}" -B "${demo_build}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${rdk_prefix}" \
  -DBUILD_FLEXIV_RT_OSC=ON
cmake --build "${demo_build}" \
  --target flexiv_7dof_torque_osc flexiv_9dof_torque_osc -j 8

echo "Built:"
echo "  ${demo_build}/flexiv_7dof_torque_osc"
echo "  ${demo_build}/flexiv_9dof_torque_osc"
