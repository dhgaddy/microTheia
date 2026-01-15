if [ -f /tools/oss-cad-suite/environment ]; then
  source /tools/oss-cad-suite/environment
else
  export PATH="/tools/oss-cad-suite/bin:$PATH"
fi
export PATH="$PATH:/tools/sv2v-Linux:/root/.cargo/bin"