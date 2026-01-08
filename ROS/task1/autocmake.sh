#!/bin/bash

rm -rf build && mkdir build && cd build
cmake .. && make
cd ..
./build/task1/task1
# echo "build/" >> .gitignore 2>/dev/null
# git add .gitignore 2>/dev/null

read -p "Commit message: " commit
git commit -m "${commit}"

git push origin main
