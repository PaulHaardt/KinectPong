#!/bin/bash

# Function to untar Unity.tar.gz into build directory and copy .env file
untar_and_setup() {
    mkdir -p build
    tar -xzf Unity.tar.gz -C build
    cp .env build/.env
    echo "Extracted Unity.tar.gz to build directory"
}

# Function to create a tar.gz of the build directory
create_tar() {
    if [ -d "build" ]; then
        tar -czf Unity.tar.gz -C build .
        echo "Created Unity.tar.gz from build directory"
    else
        echo "Error: build directory does not exist"
        exit 1
    fi
}

# Check command line arguments
case "$1" in
    "untar")
        untar_and_setup
        ;;
    "tar")
        create_tar
        ;;
    *)
        echo "Usage: $0 [untar|tar]"
        echo "  untar: Extract Unity.tar.gz to build directory"
        echo "  tar: Create Unity.tar.gz from build directory"
        exit 1
        ;;
esac
