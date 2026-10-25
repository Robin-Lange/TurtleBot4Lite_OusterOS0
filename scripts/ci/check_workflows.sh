#!/usr/bin/env bash
# Native upstream actionlint binary, pinned by release and independently verified digest.
set -euo pipefail
ACTIONLINT_VERSION=1.7.12
ACTIONLINT_SHA256=8aca8db96f1b94770f1b0d72b6dddcb1ebb8123cb3712530b08cc387b349a3d8
if [ -n "${ACTIONLINT_BIN:-}" ]; then
    "$ACTIONLINT_BIN" -color .github/workflows/*.yml
    exit
fi
if [ "$(uname -m)" != x86_64 ]; then
    echo 'Set ACTIONLINT_BIN to a native actionlint executable on non-x86-64 machines.' >&2
    exit 1
fi
TOOLS_DIR=$(mktemp -d)
trap 'rm -rf "$TOOLS_DIR"' EXIT
ARCHIVE="${TOOLS_DIR}/actionlint.tar.gz"
curl --fail --silent --show-error --location --retry 3 \
    "https://github.com/rhysd/actionlint/releases/download/v${ACTIONLINT_VERSION}/actionlint_${ACTIONLINT_VERSION}_linux_amd64.tar.gz" \
    --output "$ARCHIVE"
printf '%s  %s\n' "$ACTIONLINT_SHA256" "$ARCHIVE" | sha256sum --check --strict
tar --no-same-owner -xzf "$ARCHIVE" -C "$TOOLS_DIR" actionlint
"${TOOLS_DIR}/actionlint" -color .github/workflows/*.yml
