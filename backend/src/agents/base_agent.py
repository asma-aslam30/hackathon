"""
Base agent class for the Agent Retrieval System.

This module defines the abstract base class for all agents in the system.
"""
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, Field
from datetime import datetime
from src.utils.logging_utils import app_logger


class AgentResponse(BaseModel):
    """Base response model for agent interactions."""

    response: str = Field(..., description="The agent's answer to the query")
    query_id: str = Field(..., description="Unique identifier for the original query")
    retrieved_chunks: List[Dict] = Field(default=[], description="List of content chunks used to generate the response")
    confidence_score: float = Field(default=0.0, ge=0.0, le=1.0, description="Confidence level of the response")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the response was generated")
    sources: List[str] = Field(default=[], description="List of sources referenced in the response")
    metadata: Dict[str, Any] = Field(default={}, description="Additional metadata for the response")


class BaseAgent(ABC):
    """Abstract base class for all agents in the system."""

    def __init__(self, name: str, description: str = "", config: Optional[Dict[str, Any]] = None):
        """
        Initialize the base agent.

        Args:
            name: Unique name for the agent
            description: Description of the agent's purpose
            config: Optional configuration dictionary
        """
        self.name = name
        self.description = description
        self.config = config or {}
        self.created_at = datetime.now()
        self.is_initialized = False

        app_logger.info(f"Initialized agent: {self.name}", agent=self.name)

    @abstractmethod
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
        pass

    @abstractmethod
    def validate_query(self, query: str) -> bool:
        """
        Validate that the query is acceptable for processing.

        Args:
            query: The query to validate

        Returns:
            bool: True if query is valid, False otherwise
        """
        pass

    async def initialize(self) -> bool:
        """
        Initialize the agent with any required resources.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            app_logger.info(f"Initializing agent: {self.name}", agent=self.name)
            result = await self._setup_resources()
            self.is_initialized = result
            return result
        except Exception as e:
            app_logger.error(f"Failed to initialize agent {self.name}: {str(e)}",
                           agent=self.name, error=str(e))
            return False

    async def cleanup(self) -> bool:
        """
        Clean up any resources used by the agent.

        Returns:
            bool: True if cleanup was successful, False otherwise
        """
        try:
            app_logger.info(f"Cleaning up agent: {self.name}", agent=self.name)
            result = await self._cleanup_resources()
            return result
        except Exception as e:
            app_logger.error(f"Failed to cleanup agent {self.name}: {str(e)}",
                           agent=self.name, error=str(e))
            return False

    @abstractmethod
    async def _setup_resources(self) -> bool:
        """Setup any required resources for the agent."""
        pass

    @abstractmethod
    async def _cleanup_resources(self) -> bool:
        """Clean up any resources used by the agent."""
        pass

    def get_agent_info(self) -> Dict[str, Any]:
        """
        Get information about the agent.

        Returns:
            Dict containing agent information
        """
        return {
            "name": self.name,
            "description": self.description,
            "created_at": self.created_at.isoformat(),
            "is_initialized": self.is_initialized,
            "config_keys": list(self.config.keys()) if self.config else []
        }

    def update_config(self, new_config: Dict[str, Any]) -> None:
        """
        Update the agent's configuration.

        Args:
            new_config: New configuration values to update
        """
        if self.config is None:
            self.config = {}

        self.config.update(new_config)
        app_logger.info(f"Updated configuration for agent: {self.name}",
                       agent=self.name, updated_keys=list(new_config.keys()))

    def get_config_value(self, key: str, default: Any = None) -> Any:
        """
        Get a configuration value by key.

        Args:
            key: Configuration key to retrieve
            default: Default value if key is not found

        Returns:
            Configuration value or default
        """
        return self.config.get(key, default)