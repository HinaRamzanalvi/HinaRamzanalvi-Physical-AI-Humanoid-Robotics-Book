---
sidebar_position: 999
---

# RAG Chatbot Integration

This page explains how to integrate the RAG Chatbot into your Docusaurus textbook.

## Overview

The RAG (Retrieval-Augmented Generation) Chatbot allows students to ask questions about the textbook content and receive answers based on the textbook material with proper citations.

## Integration Methods

### Method 1: Using the Component (Recommended)

The chatbot is available as a React component that can be easily integrated into any Docusaurus page:

```jsx
import RagChatbot from '@site/src/components/RagChatbot';

function MyPage() {
  return (
    <div>
      <h1>My Page Content</h1>
      <RagChatbot />
    </div>
  );
}
```

### Method 2: Global Integration

To add the chatbot to all pages, add it to your Docusaurus layout or create a layout wrapper that includes the chatbot component.

## Backend Setup

Make sure your backend server is running:

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

## Configuration

The frontend automatically connects to `http://localhost:8000` by default. To change this, set the `VITE_API_URL` environment variable:

```bash
# .env file
VITE_API_URL=https://your-backend-domain.com
```

## Features

- **Ask Book Mode**: Ask questions about the entire textbook content
- **Ask Selected Text Mode**: Ask questions about selected text only
- **Citations**: Responses include proper citations to textbook sections
- **Session Management**: Maintains conversation context
- **Responsive Design**: Works on desktop and mobile devices

## API Endpoints Used

- `POST /api/v1/chat/query` - Submit queries
- `GET /api/v1/health` - Health check

## Troubleshooting

### Chatbot not appearing
- Make sure the component is properly imported
- Check browser console for errors

### API connection issues
- Verify the backend server is running
- Check CORS settings in the backend
- Verify the API URL configuration

### Text selection not working
- Make sure the page allows text selection
- Check browser permissions