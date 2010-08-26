#!/bin/bash
echo '==== Vision ===='
cd vision
make cleaner
make -j4 || exit
make doc
make installlink
cd ..

echo '==== Robotic ===='
cd robotic
make cleaner
make -j4 || exit
make doc
make installlink
cd ..

echo '==== Physics ===='
cd physics/physics_bullet
make cleaner
make -j4 || exit
make doc
make installlink
cd ../..

echo '==== Integration ===='
cd integration
make cleaner
make -j4 || exit
make doc
make installlink
cd ..

