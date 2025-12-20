"""
Query models for the Agent Retrieval System API.

This module defines Pydantic models for API requests and responses.
"""
from pydantic import BaseModel, Field, validator
from typing import Optional, Dict, Any, List
from datetime import datetime
import uuid


class QueryRequest(BaseModel):
    """Model for query requests to the agent API."""

    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The user's natural language question or query"
    )
    user_id: Optional[str] = Field(
        default=None,
        pattern=r'^[a-zA-Z0-9_-]+$',
        description="Optional identifier for the requesting user"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        default={},
        description="Additional contextual information for the query"
    )
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the query was submitted"
    )

    @validator('query')
    def validate_query_length(cls, v):
        """Validate that query is not too long."""
        if len(v) > 1000:
            raise ValueError('Query must be between 1 and 1000 characters')
        return v

    @validator('user_id')
    def validate_user_id_format(cls, v):
        """Validate user ID format if provided."""
        if v is not None:
            import re
            if not re.match(r'^[a-zA-Z0-9_-]+$', v):
                raise ValueError('User ID must contain only alphanumeric characters, hyphens, and underscores')
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What are the main concepts discussed in Chapter 1?",
                "user_id": "user-12345",
                "metadata": {
                    "source": "dashboard",
                    "priority": "high"
                }
            }
        }


class RetrievedChunkResponse(BaseModel):
    """Model for retrieved content chunks in API responses."""

    id: str = Field(..., description="Unique identifier for the retrieved content chunk")
    content: str = Field(..., description="The actual text content retrieved from the vector database")
    source_url: str = Field(..., description="URL or identifier of the original source document")
    similarity_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Semantic similarity score to the original query"
    )
    metadata: Dict[str, Any] = Field(default={}, description="Additional metadata associated with the chunk")

    class Config:
        json_schema_extra = {
            "example": {
                "id": "chunk-abc123",
                "content": "The main concept discussed in this section is...",
                "source_url": "https://example.com/book/chapter1",
                "similarity_score": 0.85,
                "metadata": {
                    "chapter": 1,
                    "section": "Introduction"
                }
            }
        }


class AgentResponse(BaseModel):
    """Model for agent responses from the API."""

    response: str = Field(..., description="The agent's answer to the query")
    query_id: str = Field(..., description="Unique identifier for the original query")
    retrieved_chunks: List[RetrievedChunkResponse] = Field(
        default=[],
        description="List of content chunks used to generate the response"
    )
    confidence_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence level of the response"
    )
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the response was generated"
    )
    sources: List[str] = Field(
        default=[],
        description="List of sources referenced in the response"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "response": "The main concepts in Chapter 1 include...",
                "query_id": "query-67890",
                "retrieved_chunks": [
                    {
                        "id": "chunk-abc123",
                        "content": "The main concept discussed in this section is...",
                        "source_url": "https://example.com/book/chapter1",
                        "similarity_score": 0.85,
                        "metadata": {
                            "chapter": 1,
                            "section": "Introduction"
                        }
                    }
                ],
                "confidence_score": 0.92,
                "timestamp": "2025-12-18T10:30:05Z",
                "sources": ["source1", "source2"]
            }
        }


class ErrorResponse(BaseModel):
    """Model for error responses from the API."""

    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the error occurred"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "error": "ValidationError",
                "message": "Query must not be empty",
                "timestamp": "2025-12-18T10:30:00Z"
            }
        }


class HealthResponse(BaseModel):
    """Model for health check responses."""

    status: str = Field(..., description="Health status of the service")
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the health check was performed"
    )
    version: str = Field(default="1.0.0", description="API version")

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "timestamp": "2025-12-18T10:30:00Z",
                "version": "1.0.0"
            }
        }


class MetricsResponse(BaseModel):
    """Model for metrics responses."""

    total_queries: int = Field(..., description="Total number of queries processed")
    avg_response_time: float = Field(..., description="Average response time in seconds")
    success_rate: float = Field(..., description="Success rate as a percentage")
    active_sessions: int = Field(..., description="Number of currently active sessions")
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the metrics were collected"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "total_queries": 150,
                "avg_response_time": 2.34,
                "success_rate": 0.98,
                "active_sessions": 5,
                "timestamp": "2025-12-18T10:30:00Z"
            }
        }