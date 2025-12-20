#!/bin/bash
set -e

echo "Installing dependencies..."
npm install

echo "Building Docusaurus site..."
npm run build

echo "Setup complete!"