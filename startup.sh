#!/bin/bash

# Source oss-cad-suite environment if available
if [ -f /tools/oss-cad-suite/environment ]; then
  source /tools/oss-cad-suite/environment
else
  export PATH="/tools/oss-cad-suite/bin:$PATH"
fi

# Add sv2v and Rust to PATH (use HOME instead of hardcoded path)
export PATH="$PATH:/tools:$HOME/.cargo/bin"

echo "Development environment initialized!"
