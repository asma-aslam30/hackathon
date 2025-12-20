// Type definitions based on the OpenAPI specification

export interface QueryRequest {
  /**
   * The user's question or query
   */
  query: string;

  /**
   * Optional user identifier for tracking
   */
  userId?: string;

  /**
   * Optional session identifier for conversation context
   */
  sessionId?: string;
}

export interface RetrievedChunk {
  /**
   * Unique identifier for the retrieved content chunk
   */
  id: string;

  /**
   * The actual text content retrieved from the vector database
   */
  content: string;

  /**
   * URL or identifier of the original source document
   */
  source_url: string;

  /**
   * Semantic similarity score to the original query (0-1)
   */
  similarity_score: number;

  /**
   * Additional metadata associated with the chunk
   */
  metadata?: Record<string, any>;
}

export interface QueryResponse {
  /**
   * The AI-generated response to the query
   */
  response: string;

  /**
   * Unique identifier for the query
   */
  query_id: string;

  /**
   * The content chunks retrieved from the vector database
   */
  retrieved_chunks?: RetrievedChunk[];

  /**
   * Confidence score for the response (0-1)
   */
  confidence_score: number;

  /**
   * List of source URLs used in the response
   */
  sources?: string[];
}

export interface HealthResponse {
  /**
   * Health status of the system
   */
  status: string;

  /**
   * ISO 8601 timestamp of the health check
   */
  timestamp: string;

  /**
   * Version of the API
   */
  version: string;
}

export interface ErrorResponse {
  /**
   * Human-readable error message
   */
  detail: string;

  /**
   * ISO 8601 timestamp of when the error occurred
   */
  timestamp: string;

  /**
   * Machine-readable error code
   */
  error_code?: string;
}