# Troubleshooting Guide

## Common Issues and Solutions

### 1. Backend Connection Issues

**Problem**: "Network error: Unable to reach the server"
- **Solution**: Verify that the backend server is running and accessible at the configured URL
- **Check**: Ensure the backend is running on port 8000 (or your configured port)
- **Test**: Try accessing `http://localhost:8000/health` directly in your browser

### 2. API Timeout Errors

**Problem**: "Request timeout: The server took too long to respond"
- **Solution**: The backend may be processing the query slowly or is under heavy load
- **Check**: Verify the backend is responsive and not experiencing performance issues
- **Note**: The component has a built-in retry mechanism that will attempt up to 3 times

### 3. CORS Errors

**Problem**: "Access to fetch at ... has been blocked by CORS policy"
- **Solution**: Configure CORS settings in the backend to allow requests from your frontend origin
- **Check**: Ensure the backend's CORS middleware allows requests from your domain

### 4. Invalid API Response

**Problem**: "Invalid response format from backend"
- **Solution**: Verify that the backend API is returning responses in the expected format
- **Check**: Ensure the backend is using the correct API schema

### 5. Empty or Unhelpful Responses

**Problem**: Responses are too brief or not relevant to the question
- **Solution**: This indicates an issue with the RAG system or vector database content
- **Check**: Verify that the vector database contains relevant content for the questions being asked

## Configuration Issues

### API Base URL

- **Default**: `http://localhost:8000`
- **Override**: Set the `REACT_APP_API_BASE_URL` environment variable
- **Example**: `REACT_APP_API_BASE_URL=https://api.example.com`

### Timeout Settings

- **Default**: 30 seconds
- **Adjust**: Modify the `API_TIMEOUT` constant in `ChatbotService.ts`

## Debugging Tips

### Enable Console Logging

The component logs important information to the console:
- Network errors
- Retry attempts
- Health check results

### Network Tab Inspection

Check the browser's Network tab to:
- Verify API requests are being sent
- Confirm response format matches expectations
- Check for timeout or connection issues

### Health Check

Use the service's health check method:
```javascript
const isHealthy = await chatbotService.checkHealth();
```

## Performance Considerations

### Slow Response Times

- Check backend performance
- Verify vector database connectivity and performance
- Monitor for resource constraints

### Memory Usage

- The component maintains a message history in state
- Consider implementing pagination for long conversations

## Security Notes

- All API communication should use HTTPS in production
- Input validation prevents malicious queries
- Error messages don't expose internal system details