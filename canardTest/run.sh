#!/bin/bash
cmake --build . --target=mainM -j=12
./mainM -i ../canardTest/canardTest.txt -p ../canardTest/params.dat -o ../canardTest/ -b
#rm -rf ../../python/swept_wing/
#cp -r ../swept_wing/ ../../python/
