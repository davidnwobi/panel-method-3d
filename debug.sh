#!/bin/bash
cmake --build . --target=mainM
lldb mainM -- -i ../test_dir/0012_200.txt -p ../test_dir/params.dat -o ../test_dir/ -r
