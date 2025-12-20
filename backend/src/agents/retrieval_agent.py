"""
Retrieval agent for the Agent Retrieval System.

This module implements the retrieval agent that processes queries and retrieves relevant content.
"""
from typing import Any, Dict, List, Optional
from pydantic import BaseModel, Field
from datetime import datetime
from src.agents.base_agent import BaseAgent, AgentResponse
from src.services.retrieval_service import RetrievalService
from src.config.settings import get_settings, get_ai_client
from src.utils.logging_utils import app_logger


class RetrievedChunk(BaseModel):
    """Model representing a chunk of content retrieved from the vector database."""

    id: str = Field(..., description="Unique identifier for the retrieved content chunk")
    content: str = Field(..., description="The actual text content retrieved from the vector database")
    source_url: str = Field(..., description="URL or identifier of the original source document")
    similarity_score: float = Field(default=0.0, ge=0.0, le=1.0, description="Semantic similarity score to the original query")
    metadata: Dict[str, Any] = Field(default={}, description="Additional metadata associated with the chunk")


class RetrievalAgent(BaseAgent):
    """Agent that retrieves relevant content and generates responses based on it."""

    def __init__(self, name: str = "retrieval-agent", description: str = "Agent that retrieves and answers questions based on book content", config: Optional[Dict[str, Any]] = None):
        """
        Initialize the retrieval agent.

        Args:
            name: Name for the agent
            description: Description of the agent's purpose
            config: Optional configuration dictionary
        """
        super().__init__(name, description, config)
        self.retrieval_service: Optional[RetrievalService] = None
        self.ai_client = None
        self.settings = get_settings()

    async def _setup_resources(self) -> bool:
        """Setup required resources for the retrieval agent."""
        try:
            # Initialize AI client based on configured provider
            self.ai_client = get_ai_client()

            # Initialize retrieval service
            self.retrieval_service = RetrievalService()
            await self.retrieval_service.initialize()

            app_logger.info(f"Resources initialized for retrieval agent: {self.name}")
            return True
        except Exception as e:
            app_logger.error(f"Failed to setup resources for retrieval agent {self.name}: {str(e)}",
                           agent=self.name, error=str(e))
            return False

    async def _cleanup_resources(self) -> bool:
        """Clean up resources used by the retrieval agent."""
        try:
            if self.retrieval_service:
                await self.retrieval_service.cleanup()

            app_logger.info(f"Resources cleaned up for retrieval agent: {self.name}")
            return True
        except Exception as e:
            app_logger.error(f"Failed to cleanup resources for retrieval agent {self.name}: {str(e)}",
                           agent=self.name, error=str(e))
            return False

    def validate_query(self, query: str) -> bool:
        """
        Validate that the query is acceptable for processing.

        Args:
            query: The query to validate

        Returns:
            bool: True if query is valid, False otherwise
        """
        if not query or len(query.strip()) == 0:
            return False

        if len(query) > self.settings.max_query_length:
            return False

        return True

    async def process_query(self, query: str, user_id: Optional[str] = None, **kwargs) -> AgentResponse:
        """
        Process a query and return an agent response.

        Args:
            query: The user's query to process
            user_id: Optional user identifier
            **kwargs: Additional arguments for processing

        Returns:
            AgentResponse: The agent's response to the query
        """
        if not self.validate_query(query):
            raise ValueError(f"Invalid query: {query}")

        query_id = f"query-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{hash(query) % 10000}"

        app_logger.info(f"Processing query for agent: {self.name}",
                       agent=self.name, query_id=query_id, query=query[:100])

        # Retrieve relevant content from vector database
        retrieved_chunks = []
        if self.retrieval_service:
            retrieved_chunks = await self.retrieval_service.retrieve_similar_content(query, top_k=5)

        # Generate response using OpenAI
        response_text = await self._generate_response(query, retrieved_chunks)

        # Calculate confidence score based on similarity scores
        confidence_score = 0.0
        if retrieved_chunks:
            similarity_scores = [chunk.similarity_score for chunk in retrieved_chunks]
            confidence_score = sum(similarity_scores) / len(similarity_scores) if similarity_scores else 0.0

        # Extract sources from retrieved chunks
        sources = list(set([chunk.source_url for chunk in retrieved_chunks if chunk.source_url]))

        # Prepare retrieved chunks for response
        response_chunks = [{
            "id": chunk.id,
            "content": chunk.content,
            "source_url": chunk.source_url,
            "similarity_score": chunk.similarity_score,
            "metadata": chunk.metadata
        } for chunk in retrieved_chunks]

        # Create response
        response = AgentResponse(
            response=response_text,
            query_id=query_id,
            retrieved_chunks=response_chunks,
            confidence_score=confidence_score,
            sources=sources
        )

        app_logger.info(f"Query processed successfully for agent: {self.name}",
                       agent=self.name, query_id=query_id, response_length=len(response_text))

        return response

    async def _generate_response(self, query: str, retrieved_chunks: List[RetrievedChunk]) -> str:
        """
        Generate a response using the configured AI provider based on the query and retrieved content.

        Args:
            query: The original query
            retrieved_chunks: List of retrieved content chunks

        Returns:
            str: Generated response text
        """
        if not self.ai_client:
            raise RuntimeError("AI client not initialized")

        # Format context from retrieved chunks
        context_str = ""
        for i, chunk in enumerate(retrieved_chunks):
            context_str += f"\nContext {i+1}: {chunk.content}\n"

        # Create system message
        system_message = f"""You are an AI assistant that answers questions based on provided context.
        Your goal is to provide accurate and helpful answers using only the information provided in the context.
        If the context doesn't contain information to answer the question, say so explicitly.
        Be concise but thorough in your responses."""

        # Create user message
        user_message = f"""Context: {context_str}\n\nQuestion: {query}\n\nAnswer:"""

        try:
            if self.settings.ai_provider.lower() == 'openai':
                # Use OpenAI client
                response = self.ai_client.chat.completions.create(
                    model=self.settings.openai_model,
                    messages=[
                        {"role": "system", "content": system_message},
                        {"role": "user", "content": user_message}
                    ],
                    temperature=0.3,
                    max_tokens=1000
                )
                return response.choices[0].message.content.strip()
            elif self.settings.ai_provider.lower() == 'gemini':
                # Use Gemini client
                import google.generativeai as genai

                # Combine system and user messages for Gemini
                full_prompt = f"{system_message}\n\n{user_message}"

                response = self.ai_client.generate_content(
                    full_prompt,
                    generation_config=genai.types.GenerationConfig(
                        temperature=0.3,
                        max_output_tokens=1000
                    )
                )
                return response.text.strip()
            else:
                raise ValueError(f"Unsupported AI provider: {self.settings.ai_provider}")
        except Exception as e:
            app_logger.error(f"Error generating response with {self.settings.ai_provider}: {str(e)}",
                           agent=self.name, error=str(e))
            return f"I encountered an error processing your query: {str(e)}. Please try again later."