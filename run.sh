untar the Unity.tar.gz file in the build directory. Create this directory if it does not exist.
Also copy the root .env file to the build directory.

mkdir -p build
tar -xzf Unity.tar.gz -C build
cp .env build/.env