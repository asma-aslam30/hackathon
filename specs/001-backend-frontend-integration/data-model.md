# Data Model: Backend-Frontend Integration

## Entities

### User Query
- **id**: string (auto-generated)
- **content**: string (required, max 1000 characters)
- **timestamp**: datetime (auto-generated)
- **userId**: string (optional, for tracking)

**Validation rules**:
- Content must not be empty
- Content must be less than 1000 characters
- Timestamp is automatically set to current time

### Backend Response
- **responseId**: string (auto-generated)
- **content**: string (required)
- **queryId**: string (reference to User Query)
- **sourceInfo**: array of objects (containing source URLs and context)
- **confidenceScore**: number (0-1)
- **timestamp**: datetime (auto-generated)

**Validation rules**:
- Content must not be empty
- Confidence score must be between 0 and 1
- QueryId must reference an existing User Query

### Chat Session
- **sessionId**: string (auto-generated)
- **userId**: string (optional)
- **messages**: array of objects (containing queries and responses)
- **createdAt**: datetime (auto-generated)
- **lastActive**: datetime (auto-generated)

**Validation rules**:
- Session must have at least one message to be valid
- LastActive must be greater than or equal to createdAt

### API Communication
- **requestId**: string (auto-generated)
- **endpoint**: string (required, e.g., "/api/v1/query")
- **method**: string (required, e.g., "POST")
- **status**: number (HTTP status code)
- **timestamp**: datetime (auto-generated)
- **duration**: number (response time in milliseconds)

**Validation rules**:
- Method must be a valid HTTP method
- Status must be a valid HTTP status code
- Duration must be non-negative

## State Transitions

### Query Processing States
1. **Submitted** → Query sent from frontend to backend
2. **Processing** → Backend received query, retrieving context
3. **Generating** → Backend generating response using AI agent
4. **Completed** → Response returned to frontend
5. **Error** → Error occurred during processing

### Component States
1. **Idle** → Chatbot waiting for user input
2. **Loading** → Query submitted, waiting for response
3. **Ready** → Response received and displayed
4. **Error** → Error occurred, error message displayed

## Relationships
- User Query → Backend Response (one-to-one)
- Chat Session → User Query (one-to-many)
- API Communication → User Query (one-to-one for each request)