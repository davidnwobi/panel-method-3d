#!/bin/bash
cmake --build . --target=mainM -j=12
./mainM -i ../high_res/0012_high_res.txt -p ../high_res/params.dat -o ../high_res/ -r -b
#rm -rf ../../python/test_dir/
#cp -r ../test_dir/ ../../python/
#cp -r ../build/lhs.txt ../../python/
#cp -r ../build/rhs.txt ../../python/
