#!/bin/bash
set -e

echo "Installing dependencies in book directory..."
cd book
npm install

echo "Building Docusaurus site..."
npm run build

echo "Build completed successfully in book/build directory"
ls -la build/