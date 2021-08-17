#!/usr/bin/env bash

roscore &
roscore_pid=$!

sleep 5
cd ~/.haros/export/haros_plugin_pbt_gen

for filename in *.py; do
echo "Running tests for $(basename $filename)"
python $filename >$(basename -s ".py" $filename).log.txt 2>&1
done

kill $roscore_pid

