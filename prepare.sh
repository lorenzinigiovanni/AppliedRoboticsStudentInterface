 #!/bin/sh

rm -rf ../images
mkdir ../images

rm -rf ../state
mkdir ../state

cd build
make
