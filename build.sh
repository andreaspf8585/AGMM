#!/usr/bin/env bash

set -e  # stop on error

echo "Cleaning..."
make clean

echo "Building..."
make -j16

echo "Creating dist..."
make dist

echo "Installing..."
make install

echo "Done."