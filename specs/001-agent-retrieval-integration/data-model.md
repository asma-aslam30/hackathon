# Data Model: Agent Development with Retrieval Integration

## Entities

### QueryRequest
- **query**: string - The user's natural language question
- **user_id**: string (optional) - Identifier for the requesting user
- **metadata**: dict (optional) - Additional contextual information for the query
- **timestamp**: datetime - When the query was submitted

### AgentResponse
- **response**: string - The agent's answer to the query
- **query_id**: string - Reference to the original query
- **retrieved_chunks**: list[RetrievedChunk] - List of content chunks used to generate the response
- **confidence_score**: float - Confidence level of the response (0.0-1.0)
- **timestamp**: datetime - When the response was generated
- **sources**: list[string] - List of sources referenced in the response

### RetrievedChunk
- **id**: string - Unique identifier for the retrieved content chunk
- **content**: string - The actual text content retrieved from the vector database
- **source_url**: string - URL or identifier of the original source document
- **similarity_score**: float - Semantic similarity score to the original query (0.0-1.0)
- **metadata**: dict - Additional metadata associated with the chunk

### AgentSession
- **session_id**: string - Unique identifier for the agent session
- **user_id**: string (optional) - Identifier for the user associated with the session
- **created_at**: datetime - When the session was created
- **last_activity**: datetime - When the last query was processed in this session
- **query_history**: list[QueryRequest] - List of queries processed in this session
- **settings**: dict - Session-specific settings for the agent

## Relationships

- One `AgentSession` has many `QueryRequest` entities
- One `QueryRequest` generates one `AgentResponse`
- One `AgentResponse` contains many `RetrievedChunk` entities
- Many `QueryRequest` entities can reference the same `RetrievedChunk` if the same content is retrieved

## Validation Rules

### QueryRequest
- `query` must not be empty
- `query` length must be between 1 and 1000 characters
- `user_id` must follow UUID format if provided
- `timestamp` must be in ISO 8601 format

### AgentResponse
- `response` must not be empty
- `confidence_score` must be between 0.0 and 1.0
- `query_id` must reference an existing `QueryRequest`
- `timestamp` must be in ISO 8601 format

### RetrievedChunk
- `id` must be unique across all chunks
- `content` must not be empty
- `similarity_score` must be between 0.0 and 1.0
- `source_url` must be a valid URL format

### AgentSession
- `session_id` must be unique
- `created_at` must be before or equal to `last_activity`
- `query_history` length must not exceed 100 items

## State Transitions

### AgentSession
- `created` → `active` (when first query is processed)
- `active` → `inactive` (after 30 minutes of inactivity)
- `inactive` → `archived` (after 30 days of inactivity)

### QueryRequest
- `received` → `processing` (when agent starts processing)
- `processing` → `completed` (when agent finishes processing)
- `processing` → `failed` (if an error occurs during processing)

## Indexes

### For Performance Optimization
- Index on `RetrievedChunk.similarity_score` for fast retrieval ranking
- Index on `QueryRequest.timestamp` for chronological queries
- Index on `AgentSession.last_activity` for session cleanup
- Composite index on `AgentResponse.query_id` and `AgentResponse.timestamp` for query-response correlation

## Constraints

### Data Integrity
- Referential integrity: All foreign key references must point to existing records
- Content validation: Retrieved content must be non-empty and properly encoded
- Time consistency: Timestamps must follow chronological order within a session
- Confidence bounds: All confidence scores must be between 0.0 and 1.0