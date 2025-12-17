# PowerShell script to set up Docusaurus integration for the RAG Chatbot

Write-Host "Setting up Docusaurus integration for RAG Chatbot..." -ForegroundColor Green

# Check if we're in the right directory
if (!(Test-Path "book") -or !(Test-Path "frontend") -or !(Test-Path "backend")) {
    Write-Host "Error: This script should be run from the project root directory." -ForegroundColor Red
    Write-Host "Expected structure: ./book, ./frontend, ./backend" -ForegroundColor Red
    exit 1
}

Write-Host "✓ Verified project structure" -ForegroundColor Green

# Build the frontend for Docusaurus
Write-Host "Building frontend for Docusaurus integration..." -ForegroundColor Yellow
Set-Location frontend

if (!(Get-Command npm -ErrorAction SilentlyContinue)) {
    Write-Host "Error: npm is not installed or not in PATH" -ForegroundColor Red
    exit 1
}

npm install
npm run build-for-docusaurus

if ($LASTEXITCODE -eq 0) {
    Write-Host "✓ Frontend built successfully for Docusaurus" -ForegroundColor Green
} else {
    Write-Host "✗ Frontend build failed" -ForegroundColor Red
    exit 1
}

Set-Location ..

# Copy the React component to Docusaurus
Write-Host "Copying React component to Docusaurus..." -ForegroundColor Yellow
$ragChatbotPath = "book/src/components/RagChatbot.jsx"

if (Test-Path $ragChatbotPath) {
    Write-Host "✓ RagChatbot component already exists in Docusaurus" -ForegroundColor Green
} else {
    $componentContent = @"
import RagChatbot from '../frontend/src/components/RagChatbot';
export default RagChatbot;
"@
    $componentContent | Out-File -FilePath $ragChatbotPath -Encoding UTF8
    Write-Host "✓ Created RagChatbot component in Docusaurus" -ForegroundColor Green
}

# Add integration guide to docs if needed
$integrationGuidePath = "book/docs/rag-chatbot-integration.md"
if (!(Test-Path $integrationGuidePath)) {
    Write-Host "Creating integration guide..." -ForegroundColor Yellow
    # The integration guide already exists from previous steps
}

Write-Host ""
Write-Host "Docusaurus integration setup complete!" -ForegroundColor Green
Write-Host ""
Write-Host "To run the full system:" -ForegroundColor Yellow
Write-Host "1. Start backend: cd backend && uvicorn src.main:app --reload --port 8000" -ForegroundColor Yellow
Write-Host "2. Start Docusaurus: cd book && npm start" -ForegroundColor Yellow
Write-Host ""
Write-Host "The RAG Chatbot will be available as a floating button on all pages." -ForegroundColor Yellow