# Docusaurus Integration Guide

This guide explains how to integrate the RAG Chatbot into your Docusaurus textbook site.

## Prerequisites

- Docusaurus project set up
- Backend API running on `http://localhost:8000` (or configured endpoint)
- Node.js and npm installed

## Installation

1. Place the `RagChatbot.jsx` component in your Docusaurus `src/components/` directory
2. The component is already available at `book/src/components/RagChatbot.jsx`

## Usage

### Option 1: Add to Specific Pages

Add the chatbot to specific pages by importing the component:

```jsx
import RagChatbot from '@site/src/components/RagChatbot';

// In your MDX file or React component
export default function MyPage() {
  return (
    <div>
      <h1>Page Content</h1>
      {/* Your page content */}
      <RagChatbot />
    </div>
  );
}
```

### Option 2: Global Integration (Recommended)

To add the chatbot to all pages, you can create a layout wrapper or modify your theme.

1. Create a layout wrapper in `src/theme/Layout/index.js`:

```jsx
import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import RagChatbot from '@site/src/components/RagChatbot';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <RagChatbot />
      </OriginalLayout>
    </>
  );
}
```

2. The chatbot will appear as a floating button on all pages.

## Configuration

The chatbot connects to the backend API automatically. The default API URL is `http://localhost:8000`. To change this, you would need to rebuild the component with the correct API URL.

## Environment Variables

If you're building the frontend separately, you can configure the API URL:

```bash
VITE_API_URL=https://your-backend-domain.com
```

## Running the Complete System

1. Start the backend:
```bash
cd backend
pip install -r requirements.txt
uvicorn src.main:app --reload --port 8000
```

2. Start Docusaurus:
```bash
cd book
npm start
```

The chatbot will be available on all pages as a floating button in the bottom-right corner.

## API Connection

The frontend connects to the backend API at the configured endpoint. All communication happens through the following endpoints:

- `POST /api/v1/chat/query` - Submit questions
- `GET /api/v1/chat/session/{session_id}` - Get session history
- `POST /api/v1/chat/session/{session_id}/clear` - Clear session
- `GET /api/v1/health` - Health check

## Troubleshooting

- If the chatbot doesn't appear, check the browser console for errors
- If API calls fail, verify the backend server is running
- Make sure CORS settings allow requests from your Docusaurus domain