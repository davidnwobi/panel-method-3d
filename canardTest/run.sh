#!/bin/bash

cmake --build . --target=mainM -j=12
cdir=../canardTest
exec=mainM
cp_file=0

./$exec -i $cdir/canardTest.txt -p $cdir/params.dat -o $cdir/ -r -b

if [ $cp_file -eq 1 ] ; then
  cp -r lhs.txt $cdir
  cp -r rhs.txt $cdir
  cp -r solution.txt $cdir
fi  
