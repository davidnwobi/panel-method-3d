#!/bin/bash
cmake --build . --target=mainM -j=12
./mainM -i ../swept_wing/swept_wing.txt -p ../swept_wing/params.dat -o ../swept_wing/ -r -b
#rm -rf ../../python/swept_wing/
#cp -r ../swept_wing/ ../../python/
