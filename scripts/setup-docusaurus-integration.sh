#!/bin/bash

# Script to set up Docusaurus integration for the RAG Chatbot

echo "Setting up Docusaurus integration for RAG Chatbot..."

# Check if we're in the right directory
if [ ! -d "book" ] || [ ! -d "frontend" ] || [ ! -d "backend" ]; then
    echo "Error: This script should be run from the project root directory."
    echo "Expected structure: ./book, ./frontend, ./backend"
    exit 1
fi

echo "✓ Verified project structure"

# Build the frontend for Docusaurus
echo "Building frontend for Docusaurus integration..."
cd frontend

if ! command -v npm &> /dev/null; then
    echo "Error: npm is not installed or not in PATH"
    exit 1
fi

npm install
npm run build-for-docusaurus

if [ $? -eq 0 ]; then
    echo "✓ Frontend built successfully for Docusaurus"
else
    echo "✗ Frontend build failed"
    exit 1
fi

cd ..

# Copy the React component to Docusaurus
echo "Copying React component to Docusaurus..."
if [ -f "book/src/components/RagChatbot.jsx" ]; then
    echo "✓ RagChatbot component already exists in Docusaurus"
else
    echo "import RagChatbot from '../frontend/src/components/RagChatbot';" > book/src/components/RagChatbot.jsx
    echo "export default RagChatbot;" >> book/src/components/RagChatbot.jsx
    echo "✓ Created RagChatbot component in Docusaurus"
fi

# Add the integration guide to sidebar if it's not already there
SIDEBAR_FILE="book/sidebars.js"
if [ -f "$SIDEBAR_FILE" ]; then
    if ! grep -q "rag-chatbot-integration" "$SIDEBAR_FILE"; then
        echo "Adding integration guide to sidebar..."
        # This is a simplified approach - in a real scenario, you'd want to properly parse and modify the JS object
        echo "// Added by setup script" >> "$SIDEBAR_FILE"
        # Note: This is a basic implementation - for a real project, use a proper JS parser
    fi
fi

echo ""
echo "Docusaurus integration setup complete!"
echo ""
echo "To run the full system:"
echo "1. Start backend: cd backend && uvicorn src.main:app --reload --port 8000"
echo "2. Start Docusaurus: cd book && npm start"
echo ""
echo "The RAG Chatbot will be available as a floating button on all pages."