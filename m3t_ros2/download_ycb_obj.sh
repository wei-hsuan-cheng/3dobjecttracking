#!/usr/bin/env bash

# Download the YCB 003_cracker_box Google 16k textured model.
# Run this script from the m3t_ros2 package directory.

set -euo pipefail

object_dir="assets/cracker_box"
source_url="https://ycb-benchmarks.s3.amazonaws.com/data/google/003_cracker_box_google_16k.tgz"
temp_dir="$(mktemp -d)"

cleanup() {
  rm -rf "${temp_dir}"
}
trap cleanup EXIT

curl -fL "${source_url}" -o "${temp_dir}/cracker_box.tgz"
tar -xzf "${temp_dir}/cracker_box.tgz" -C "${temp_dir}"

obj_file="$(find "${temp_dir}" -type f -name textured.obj -print -quit)"
mtl_file="$(find "${temp_dir}" -type f -name textured.mtl -print -quit)"
texture_file="$(find "${temp_dir}" -type f -name texture_map.png -print -quit)"

test -n "${obj_file}"
test -n "${mtl_file}"
test -n "${texture_file}"

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
# YCB cracker box mesh

\`model.obj\`, \`textured.mtl\`, and \`texture_map.png\` are the Google 16k textured model for YCB object \`003_cracker_box\`, downloaded from \`${source_url}\`.

The YCB data is licensed under Creative Commons Attribution 4.0 International (CC BY 4.0). The original \`textured.obj\` is retained as \`model.obj\` to match the package-wide asset naming convention. Trailing whitespace is removed from the MTL so RViz/OGRE resolves its texture filename correctly.

SHA-256:

- \`model.obj\`: \`${obj_sha}\`
- \`textured.mtl\`: \`${mtl_sha}\`
- \`texture_map.png\`: \`${texture_sha}\`
EOF

echo "Downloaded YCB 003_cracker_box assets to ${object_dir}:"
ls -lh "${object_dir}"