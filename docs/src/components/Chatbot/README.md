# Chatbot Component

The Chatbot component provides an interface for users to interact with the RAG (Retrieval-Augmented Generation) system. Users can ask questions about book content and receive AI-generated responses based on retrieved information from the vector database.

## Features

- Real-time query submission and response display
- Loading indicators during query processing
- Source information and confidence scores for responses
- Error handling for network and server issues
- Retry mechanism for failed requests
- Responsive design compatible with Docusaurus theme

## Usage

The component can be integrated into any Docusaurus page by importing and using it as a React component.

## API Integration

The component communicates with the backend API at `/api/v1/query` endpoint using the ChatbotService, which handles:
- Query validation (max 1000 characters)
- API request and response handling
- Error management
- Retry logic for transient failures

## Security

- Uses HTTPS for all API communications
- Implements proper input validation
- Sanitizes output to prevent XSS attacks

## Accessibility

- Proper ARIA labels and semantic HTML
- Keyboard navigation support
- Screen reader compatibility

## Performance

- Implements timeout handling (30 seconds)
- Optimized rendering with React hooks
- Efficient state management