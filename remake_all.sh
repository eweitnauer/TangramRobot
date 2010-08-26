#!/bin/bash
echo '==== Vision ===='
make cleaner -C vision
make -j4  -C vision || exit
make doc -C vision
make installlink -C vision

echo '==== Physics ===='
make cleaner -C physics
make -j4 -C physics || exit
make doc -C physics
make installlink -C physics

echo '==== Integration ===='
make cleaner -C integration
make -j4 -C integration || exit
make doc -C integration
make installlink -C integration
