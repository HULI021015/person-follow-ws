#!/usr/bin/env bash
# Source this file before importing Dahua's NetSDK Python package.

NETSDK_LIB_DIR="$(python3 -c 'import importlib.util, pathlib; s=importlib.util.find_spec("NetSDK"); print(pathlib.Path(next(iter(s.submodule_search_locations))) / "Libs" / "linux64" if s and s.submodule_search_locations else "")')"
if [[ -z "${NETSDK_LIB_DIR}" || ! -d "${NETSDK_LIB_DIR}" ]]; then
  echo "未找到大华 NetSDK。请先运行 scripts/install_dependencies.sh。" >&2
  return 1 2>/dev/null || exit 1
fi

export LD_LIBRARY_PATH="${NETSDK_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
unset NETSDK_LIB_DIR
