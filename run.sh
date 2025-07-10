#!/bin/bash

echo "Copying .env files to Unity and src/udp_server directories..."
cp .env src/udp_server/.env
echo ".Successfully copied .env files."

echo "Starting UDP server..."
cd src/udp_server
if [ -f "build/simple_server" ]; then
    ./build/simple_server
else
    echo "Build not found. Please build the UDP server first."
    exit 1
fi

echo "UDP server started. Now running Unity..."

cd ../../
tar -xvf Unity.tar.gz
cd Unity
if [ -f "KinectPong.exe" ]; then
    ./KinectPong.x86_64
else
    echo "KinectPong executable not found. Please ensure it is built and available."
    exit 1
fi