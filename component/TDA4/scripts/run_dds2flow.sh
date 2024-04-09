a=$(uname -m)
if [ $a = "x86_64" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./build/linux/runtime
    ./build/linux/bin/dds_to_flow ./param/dds2flow.json
elif [ $a = "aarch64" ]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/app/algo/apa/lib
    /data/mviz_upstream_tools/bin/dds_to_flow   /data/mviz_upstream_tools/config/dds2flow.json
else
    echo "other arch"
fi
