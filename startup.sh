#!/bin/bash
set -e

# Only source oss-cad-suite if it exists
if [ -f /tools/oss-cad-suite/environment ]; then
  . /tools/oss-cad-suite/environment
fi

# Extend PATH safely
export PATH="/tools:/home/vscode/.cargo/bin:$PATH"

echo "Development environment initialized!"
