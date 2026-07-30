#!/usr/bin/env bash

# Download a YCB Google 16k textured model.
# Run this script from the m3t_ros2 package directory.
#
# Usage:
#   ./download_ycb_obj.sh <ycb_object_id>
#
# Examples:
#   ./download_ycb_obj.sh 003_cracker_box
#   ./download_ycb_obj.sh 006_mustard_bottle
#
# The local asset name is derived by removing the leading three-digit YCB ID
# and underscore. For example, 006_mustard_bottle becomes mustard_bottle.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./download_ycb_obj.sh <ycb_object_id>

Arguments:
  ycb_object_id  Official YCB name, for example 003_cracker_box

Examples:
  ./download_ycb_obj.sh 003_cracker_box
  ./download_ycb_obj.sh 006_mustard_bottle
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if (( $# != 1 )); then
  usage >&2
  exit 2
fi

ycb_object_id="$1"

if [[ ! "${ycb_object_id}" =~ ^[0-9]{3}_[a-z0-9_]+$ ]]; then
  echo "Error: ycb_object_id must look like 003_cracker_box." >&2
  exit 2
fi

asset_name="${ycb_object_id#???_}"
object_label="${asset_name//_/ }"
object_dir="assets/${asset_name}"
source_url="https://ycb-benchmarks.s3.amazonaws.com/data/google/${ycb_object_id}_google_16k.tgz"
temp_dir="$(mktemp -d)"

cleanup() {
  rm -rf "${temp_dir}"
}
trap cleanup EXIT

archive_path="${temp_dir}/${ycb_object_id}_google_16k.tgz"
curl -fL "${source_url}" -o "${archive_path}"
tar -xzf "${archive_path}" -C "${temp_dir}"

obj_file="$(find "${temp_dir}" -type f -name textured.obj -print -quit)"
mtl_file="$(find "${temp_dir}" -type f -name textured.mtl -print -quit)"
texture_file="$(find "${temp_dir}" -type f -name texture_map.png -print -quit)"

if [[ -z "${obj_file}" || -z "${mtl_file}" || -z "${texture_file}" ]]; then
  echo "Error: the archive does not contain textured.obj, textured.mtl, and texture_map.png." >&2
  exit 1
fi

mkdir -p "${object_dir}"
install -m 644 "${obj_file}" "${object_dir}/model.obj"
install -m 644 "${texture_file}" "${object_dir}/texture_map.png"

LC_ALL=C sed 's/[[:space:]]*$//' "${mtl_file}" \
  > "${object_dir}/textured.mtl"
chmod 644 "${object_dir}/textured.mtl"

obj_sha="$(sha256sum "${object_dir}/model.obj" | awk '{print $1}')"
mtl_sha="$(sha256sum "${object_dir}/textured.mtl" | awk '{print $1}')"
texture_sha="$(sha256sum "${object_dir}/texture_map.png" | awk '{print $1}')"

cat > "${object_dir}/SOURCE.md" <<EOF
# YCB ${object_label} mesh

\`model.obj\`, \`textured.mtl\`, and \`texture_map.png\` are the Google 16k textured model for YCB object \`${ycb_object_id}\`, downloaded from \`${source_url}\`.

The YCB data is licensed under Creative Commons Attribution 4.0 International (CC BY 4.0). The original \`textured.obj\` is retained as \`model.obj\` to match the package-wide asset naming convention. Trailing whitespace is removed from the MTL so RViz/OGRE resolves its texture filename correctly.

SHA-256:

- \`model.obj\`: \`${obj_sha}\`
- \`textured.mtl\`: \`${mtl_sha}\`
- \`texture_map.png\`: \`${texture_sha}\`
EOF

echo "Downloaded YCB ${ycb_object_id} assets to ${object_dir}:"
ls -lh "${object_dir}"