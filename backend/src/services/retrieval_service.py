"""
Retrieval service for the Agent Retrieval System.

This module provides functionality for retrieving relevant content from the vector database.
"""
from typing import Any, Dict, List, Optional
from src.services.qdrant_service import QdrantService, QdrantDocument
from src.utils.logging_utils import app_logger
from pydantic import BaseModel, Field
from datetime import datetime
import asyncio
from openai import OpenAI
from src.config.settings import get_settings


class RetrievedChunk(BaseModel):
    """Model representing a chunk of content retrieved from the vector database."""

    id: str = Field(..., description="Unique identifier for the retrieved content chunk")
    content: str = Field(..., description="The actual text content retrieved from the vector database")
    source_url: str = Field(..., description="URL or identifier of the original source document")
    similarity_score: float = Field(default=0.0, ge=0.0, le=1.0, description="Semantic similarity score to the original query")
    metadata: Dict[str, Any] = Field(default={}, description="Additional metadata associated with the chunk")


class RetrievalService:
    """Service class for retrieving relevant content based on queries."""

    def __init__(self):
        """Initialize the retrieval service."""
        self.qdrant_service = QdrantService()
        self.settings = get_settings()
        self.openai_client: Optional[OpenAI] = None
        self.is_initialized = False

    async def initialize(self) -> bool:
        """
        Initialize the retrieval service with required resources.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            app_logger.info("Initializing retrieval service...")

            # Initialize Qdrant service
            qdrant_success = await self.qdrant_service.initialize()
            if not qdrant_success:
                raise RuntimeError("Failed to initialize Qdrant service")

            # Initialize OpenAI client for embedding generation
            self.openai_client = self.settings.get_openai_client()

            self.is_initialized = True
            app_logger.info("Retrieval service initialized successfully")
            return True
        except Exception as e:
            app_logger.error(f"Failed to initialize retrieval service: {str(e)}", error=str(e))
            self.is_initialized = False
            return False

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate an embedding for the given text using OpenAI.

        Args:
            text: Text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        if not self.openai_client:
            raise RuntimeError("OpenAI client not initialized")

        try:
            response = self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-ada-002"  # Using a common embedding model
            )

            embedding = response.data[0].embedding
            app_logger.debug(f"Generated embedding for text of length {len(text)}",
                            text_length=len(text), embedding_length=len(embedding))
            return embedding
        except Exception as e:
            app_logger.error(f"Failed to generate embedding for text: {str(e)}",
                            text_length=len(text) if text else 0, error=str(e))
            raise

    async def retrieve_similar_content(self, query: str, top_k: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[RetrievedChunk]:
        """
        Retrieve content similar to the query from the vector database.

        Args:
            query: Query text to find similar content for
            top_k: Number of similar content pieces to return
            filters: Optional filters to apply to the search

        Returns:
            List of RetrievedChunk objects containing similar content
        """
        if not self.is_initialized:
            raise RuntimeError("Retrieval service not initialized")

        app_logger.info(f"Retrieving similar content for query: {query[:100]}...",
                       query_length=len(query), top_k=top_k)

        try:
            # Generate embedding for the query
            query_embedding = await self.generate_embedding(query)

            # Search for similar content in Qdrant
            similar_docs = await self.qdrant_service.search_similar(
                query_vector=query_embedding,
                top_k=top_k,
                filters=filters
            )

            # Convert QdrantDocuments to RetrievedChunks
            retrieved_chunks = []
            for i, doc in enumerate(similar_docs):
                # Calculate similarity score based on rank (higher rank = lower score)
                # In a real implementation, this would be based on the actual distance/similarity returned by Qdrant
                similarity_score = max(0.0, 1.0 - (i / top_k))  # Simple ranking-based score

                chunk = RetrievedChunk(
                    id=doc.id,
                    content=doc.content,
                    source_url=doc.source_url,
                    similarity_score=similarity_score,
                    metadata=doc.metadata
                )
                retrieved_chunks.append(chunk)

            app_logger.info(f"Retrieved {len(retrieved_chunks)} similar content chunks",
                           retrieved_count=len(retrieved_chunks), query_length=len(query))
            return retrieved_chunks
        except Exception as e:
            app_logger.error(f"Failed to retrieve similar content for query: {str(e)}",
                            query_length=len(query) if query else 0, error=str(e))
            return []

    async def retrieve_by_source(self, source_url: str, top_k: int = 5) -> List[RetrievedChunk]:
        """
        Retrieve content from a specific source.

        Args:
            source_url: URL of the source to retrieve content from
            top_k: Number of content pieces to return

        Returns:
            List of RetrievedChunk objects from the specified source
        """
        if not self.is_initialized:
            raise RuntimeError("Retrieval service not initialized")

        try:
            # Filter by source URL
            filters = {"source_url": source_url}

            # For now, we'll retrieve all documents and filter by source
            # In a real implementation, this would be done at the database level
            all_docs = await self.qdrant_service.search_similar(
                query_vector=[0.0] * 1536,  # Dummy vector to get all docs with filters
                top_k=100,  # Get more than needed to account for filtering
                filters=filters
            )

            # Filter to only the requested source and limit to top_k
            filtered_docs = [doc for doc in all_docs if doc.source_url == source_url][:top_k]

            # Convert to RetrievedChunks
            retrieved_chunks = []
            for i, doc in enumerate(filtered_docs):
                chunk = RetrievedChunk(
                    id=doc.id,
                    content=doc.content,
                    source_url=doc.source_url,
                    similarity_score=1.0,  # Exact source match
                    metadata=doc.metadata
                )
                retrieved_chunks.append(chunk)

            app_logger.info(f"Retrieved {len(retrieved_chunks)} content chunks from source: {source_url}",
                           retrieved_count=len(retrieved_chunks), source_url=source_url)
            return retrieved_chunks
        except Exception as e:
            app_logger.error(f"Failed to retrieve content from source {source_url}: {str(e)}",
                            source_url=source_url, error=str(e))
            return []

    async def validate_content_relevance(self, query: str, content: str, threshold: float = 0.7) -> bool:
        """
        Validate if the content is relevant to the query based on semantic similarity.

        Args:
            query: Query to check relevance against
            content: Content to validate
            threshold: Minimum similarity threshold for relevance

        Returns:
            bool: True if content is relevant to query, False otherwise
        """
        if not self.is_initialized:
            raise RuntimeError("Retrieval service not initialized")

        try:
            # Generate embeddings for both query and content
            query_embedding = await self.generate_embedding(query)
            content_embedding = await self.generate_embedding(content)

            # Calculate cosine similarity between embeddings
            similarity = self._cosine_similarity(query_embedding, content_embedding)

            is_relevant = similarity >= threshold

            app_logger.debug(f"Content relevance check: similarity={similarity:.3f}, threshold={threshold}, relevant={is_relevant}",
                            similarity=similarity, threshold=threshold, is_relevant=is_relevant)
            return is_relevant
        except Exception as e:
            app_logger.error(f"Failed to validate content relevance: {str(e)}", error=str(e))
            return False

    def _cosine_similarity(self, vec_a: List[float], vec_b: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors.

        Args:
            vec_a: First vector
            vec_b: Second vector

        Returns:
            float: Cosine similarity score between -1 and 1
        """
        import numpy as np

        # Convert to numpy arrays
        a = np.array(vec_a)
        b = np.array(vec_b)

        # Calculate cosine similarity
        dot_product = np.dot(a, b)
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)

        if norm_a == 0 or norm_b == 0:
            return 0.0

        similarity = dot_product / (norm_a * norm_b)
        return float(similarity)

    async def get_content_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the content in the vector database.

        Returns:
            Dict containing content statistics
        """
        if not self.is_initialized:
            raise RuntimeError("Retrieval service not initialized")

        try:
            collection_info = await self.qdrant_service.get_collection_info()
            if not collection_info:
                return {"error": "Could not retrieve collection info"}

            stats = {
                "collection_name": self.settings.qdrant_collection_name,
                "total_documents": collection_info.get("points_count", 0),
                "indexed": True,
                "last_updated": datetime.now().isoformat()
            }

            app_logger.info("Retrieved content statistics", **stats)
            return stats
        except Exception as e:
            app_logger.error(f"Failed to get content statistics: {str(e)}", error=str(e))
            return {"error": str(e)}

    async def cleanup(self) -> bool:
        """
        Clean up resources used by the service.

        Returns:
            bool: True if cleanup was successful, False otherwise
        """
        try:
            # Clean up Qdrant service
            qdrant_cleanup = await self.qdrant_service.cleanup()

            self.is_initialized = False
            app_logger.info("Retrieval service cleaned up successfully")
            return qdrant_cleanup
        except Exception as e:
            app_logger.error(f"Failed to cleanup retrieval service: {str(e)}", error=str(e))
            return False