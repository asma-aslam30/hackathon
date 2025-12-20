"""
Qdrant service for the Agent Retrieval System.

This module provides functionality for interacting with the Qdrant vector database.
"""
from typing import Any, Dict, List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.conversions import common_types
from src.config.settings import get_settings
from src.utils.logging_utils import app_logger
from pydantic import BaseModel, Field
from datetime import datetime
import asyncio


class QdrantDocument(BaseModel):
    """Model representing a document stored in Qdrant."""

    id: str = Field(..., description="Unique identifier for the document")
    content: str = Field(..., description="The actual content of the document")
    source_url: str = Field(..., description="URL or identifier of the original source document")
    metadata: Dict[str, Any] = Field(default={}, description="Additional metadata associated with the document")
    vector: Optional[List[float]] = Field(default=None, description="Vector representation of the content")


class QdrantService:
    """Service class for interacting with Qdrant vector database."""

    def __init__(self):
        """Initialize the Qdrant service."""
        self.client: Optional[QdrantClient] = None
        self.settings = get_settings()
        self.collection_name = self.settings.qdrant_collection_name
        self.is_connected = False

    async def initialize(self) -> bool:
        """
        Initialize the connection to Qdrant.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            app_logger.info("Initializing Qdrant connection...",
                           collection=self.collection_name,
                           host=self.settings.qdrant_host)

            # Initialize Qdrant client
            self.client = self.settings.get_qdrant_client()

            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                app_logger.warning(f"Collection {self.collection_name} does not exist, creating it...",
                                  collection=self.collection_name)
                await self._create_collection()

            self.is_connected = True
            app_logger.info("Qdrant connection initialized successfully",
                           collection=self.collection_name)
            return True
        except Exception as e:
            app_logger.error(f"Failed to initialize Qdrant connection: {str(e)}",
                           collection=self.collection_name, error=str(e))
            self.is_connected = False
            return False

    async def _create_collection(self) -> bool:
        """
        Create the Qdrant collection with appropriate configuration.

        Returns:
            bool: True if collection was created successfully, False otherwise
        """
        try:
            # Default vector size - this would typically be based on the embedding model used
            # For now, we'll use 1536 which is common for OpenAI embeddings
            vector_size = 1536

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )

            app_logger.info(f"Created Qdrant collection: {self.collection_name}",
                           collection=self.collection_name, vector_size=vector_size)
            return True
        except Exception as e:
            app_logger.error(f"Failed to create Qdrant collection {self.collection_name}: {str(e)}",
                           collection=self.collection_name, error=str(e))
            return False

    async def store_document(self, document: QdrantDocument) -> bool:
        """
        Store a document in the Qdrant collection.

        Args:
            document: The document to store

        Returns:
            bool: True if storage was successful, False otherwise
        """
        if not self.is_connected or not self.client:
            raise RuntimeError("Qdrant service not initialized")

        try:
            if document.vector is None:
                app_logger.warning("Document has no vector, skipping storage",
                                  document_id=document.id)
                return False

            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=document.id,
                        vector=document.vector,
                        payload={
                            "content": document.content,
                            "source_url": document.source_url,
                            "metadata": document.metadata,
                            "timestamp": datetime.now().isoformat()
                        }
                    )
                ]
            )

            app_logger.info(f"Stored document in Qdrant: {document.id}",
                           document_id=document.id, collection=self.collection_name)
            return True
        except Exception as e:
            app_logger.error(f"Failed to store document {document.id} in Qdrant: {str(e)}",
                           document_id=document.id, collection=self.collection_name, error=str(e))
            return False

    async def batch_store_documents(self, documents: List[QdrantDocument]) -> Dict[str, Any]:
        """
        Store multiple documents in the Qdrant collection.

        Args:
            documents: List of documents to store

        Returns:
            Dict containing success count and error details
        """
        if not self.is_connected or not self.client:
            raise RuntimeError("Qdrant service not initialized")

        success_count = 0
        errors = []

        for document in documents:
            try:
                result = await self.store_document(document)
                if result:
                    success_count += 1
            except Exception as e:
                errors.append({
                    "document_id": document.id,
                    "error": str(e)
                })

        result = {
            "total_documents": len(documents),
            "successful": success_count,
            "failed": len(documents) - success_count,
            "errors": errors
        }

        app_logger.info(f"Batch stored documents in Qdrant",
                       collection=self.collection_name, **result)
        return result

    async def retrieve_by_id(self, document_id: str) -> Optional[QdrantDocument]:
        """
        Retrieve a document by its ID.

        Args:
            document_id: The ID of the document to retrieve

        Returns:
            QdrantDocument if found, None otherwise
        """
        if not self.is_connected or not self.client:
            raise RuntimeError("Qdrant service not initialized")

        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[document_id]
            )

            if not records:
                return None

            record = records[0]
            return QdrantDocument(
                id=record.id,
                content=record.payload.get("content", ""),
                source_url=record.payload.get("source_url", ""),
                metadata=record.payload.get("metadata", {}),
                vector=record.vector if hasattr(record, 'vector') else None
            )
        except Exception as e:
            app_logger.error(f"Failed to retrieve document {document_id} from Qdrant: {str(e)}",
                           document_id=document_id, collection=self.collection_name, error=str(e))
            return None

    async def search_similar(self, query_vector: List[float], top_k: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[QdrantDocument]:
        """
        Search for documents similar to the query vector.

        Args:
            query_vector: Vector to search for similarity
            top_k: Number of similar documents to return
            filters: Optional filters to apply to the search

        Returns:
            List of similar documents
        """
        if not self.is_connected or not self.client:
            raise RuntimeError("Qdrant service not initialized")

        try:
            # Build filters if provided
            search_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    if isinstance(value, list):
                        conditions.append(models.FieldCondition(
                            key=key,
                            match=models.MatchAny(any=value)
                        ))
                    else:
                        conditions.append(models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        ))

                if conditions:
                    search_filter = models.Filter(must=conditions)

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                query_filter=search_filter
            )

            documents = []
            for result in results:
                documents.append(QdrantDocument(
                    id=result.id,
                    content=result.payload.get("content", ""),
                    source_url=result.payload.get("source_url", ""),
                    metadata=result.payload.get("metadata", {}),
                    vector=result.vector if hasattr(result, 'vector') else None
                ))

            app_logger.info(f"Found {len(documents)} similar documents in Qdrant",
                           collection=self.collection_name, query_size=len(query_vector), top_k=top_k)
            return documents
        except Exception as e:
            app_logger.error(f"Failed to search similar documents in Qdrant: {str(e)}",
                           collection=self.collection_name, error=str(e))
            return []

    async def get_collection_info(self) -> Optional[Dict[str, Any]]:
        """
        Get information about the collection.

        Returns:
            Dict containing collection information
        """
        if not self.is_connected or not self.client:
            raise RuntimeError("Qdrant service not initialized")

        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors_config.size,
                "vector_size": collection_info.config.params.vectors_config.size if hasattr(collection_info.config.params, 'vectors_config') else None,
                "points_count": collection_info.points_count,
                "config": collection_info.config.dict() if hasattr(collection_info.config, 'dict') else {}
            }
        except Exception as e:
            app_logger.error(f"Failed to get collection info for {self.collection_name}: {str(e)}",
                           collection=self.collection_name, error=str(e))
            return None

    async def cleanup(self) -> bool:
        """
        Clean up resources used by the service.

        Returns:
            bool: True if cleanup was successful, False otherwise
        """
        try:
            if self.client:
                # Close the client connection if possible
                if hasattr(self.client, '_client') and hasattr(self.client._client, 'close'):
                    await self.client._client.close()
                elif hasattr(self.client, 'close'):
                    self.client.close()

            self.is_connected = False
            app_logger.info("Qdrant service cleaned up successfully",
                           collection=self.collection_name)
            return True
        except Exception as e:
            app_logger.error(f"Failed to cleanup Qdrant service: {str(e)}",
                           collection=self.collection_name, error=str(e))
            return False

    async def ping(self) -> bool:
        """
        Check if the Qdrant service is reachable.

        Returns:
            bool: True if service is reachable, False otherwise
        """
        try:
            if not self.client:
                return False

            # Try to get collection info as a simple ping
            collections = self.client.get_collections()
            return len(collections.collections) >= 0
        except Exception:
            return False