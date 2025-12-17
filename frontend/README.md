# RAG Chatbot Frontend

This is the frontend for the RAG Chatbot for the Physical AI & Humanoid Robotics Textbook.

## Features

- Floating chat interface
- Text selection functionality
- Session management
- Citation display
- Responsive design

## Tech Stack

- React 18
- TypeScript
- CSS Modules
- Vite

## Installation

1. Make sure you have Node.js (v16 or higher) installed
2. Install dependencies:

```bash
npm install
```

## Development

To run the development server:

```bash
npm run dev
```

This will start the frontend on `http://localhost:3000`.

## Building

To create a production build:

```bash
npm run build
```

## API Integration

The frontend communicates with the backend API at `http://localhost:8000/api/v1/chat/query`.

Make sure the backend server is running before starting the frontend.

## Environment Variables

No environment variables are required for the frontend.

## Project Structure

```
src/
├── components/     # React components
├── styles/         # Global styles
├── utils/          # Utility functions
├── types/          # TypeScript type definitions
└── App.tsx         # Main application component
```