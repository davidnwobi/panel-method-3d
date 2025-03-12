#!/bin/bash
cmake --build . --target=mainM
./mainM -i ../nlr_wing/nlr_wing.txt -p ../nlr_wing/params.dat -o ../nlr_wing -r
rm -rf ../../python/nlr_wing/
cp -r ../nlr_wing/ ../../python/
cp -r ../build/lhs.txt ../../python/
