#!/bin/bash
cmake --build . --target=mainM -j=12
./mainM -i ../test_dir/0012_200.txt -p ../test_dir/params.dat -o ../test_dir/ -r -b
rm -rf ../../python/test_dir/
cp -r ../test_dir/ ../../python/
cp -r ../build/lhs.txt ../../python/
cp -r ../build/rhs.txt ../../python/
