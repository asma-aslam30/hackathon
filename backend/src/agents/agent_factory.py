"""
Agent factory for the Agent Retrieval System.

This module provides a factory for creating different types of agents.
"""
from typing import Dict, Type, Any, Optional
from src.agents.base_agent import BaseAgent
from src.agents.retrieval_agent import RetrievalAgent
from src.utils.logging_utils import app_logger


class AgentFactory:
    """Factory class for creating and managing different types of agents."""

    # Registry of available agent types
    _agent_registry: Dict[str, Type[BaseAgent]] = {}

    @classmethod
    def register_agent(cls, name: str, agent_class: Type[BaseAgent]):
        """
        Register an agent class with the factory.

        Args:
            name: Name to register the agent class under
            agent_class: The agent class to register
        """
        cls._agent_registry[name] = agent_class
        app_logger.info(f"Registered agent class: {name}", agent=name, class=agent_class.__name__)

    @classmethod
    def create_agent(cls, agent_type: str, name: str, description: str = "", config: Optional[Dict[str, Any]] = None) -> BaseAgent:
        """
        Create an instance of the specified agent type.

        Args:
            agent_type: Type of agent to create
            name: Name for the agent instance
            description: Description of the agent's purpose
            config: Optional configuration for the agent

        Returns:
            BaseAgent: Instance of the requested agent type

        Raises:
            ValueError: If the agent type is not registered
        """
        if agent_type not in cls._agent_registry:
            available_types = list(cls._agent_registry.keys())
            error_msg = f"Unknown agent type: {agent_type}. Available types: {available_types}"
            app_logger.error(error_msg, requested_type=agent_type, available_types=available_types)
            raise ValueError(error_msg)

        agent_class = cls._agent_registry[agent_type]
        agent_instance = agent_class(name=name, description=description, config=config)

        app_logger.info(f"Created agent instance: {name}", agent=name, type=agent_type)
        return agent_instance

    @classmethod
    def get_available_agent_types(cls) -> Dict[str, str]:
        """
        Get a list of available agent types with their descriptions.

        Returns:
            Dict mapping agent type names to their class names
        """
        return {name: agent_class.__name__ for name, agent_class in cls._agent_registry.items()}


# Register the default agents
AgentFactory.register_agent("retrieval", RetrievalAgent)

# For future expansion, other agent types could be registered here:
# AgentFactory.register_agent("chat", ChatAgent)
# AgentFactory.register_agent("analysis", AnalysisAgent)
# AgentFactory.register_agent("summarization", SummarizationAgent)


async def get_default_retrieval_agent() -> RetrievalAgent:
    """
    Get the default retrieval agent instance.

    Returns:
        RetrievalAgent: Default retrieval agent instance
    """
    agent = AgentFactory.create_agent(
        agent_type="retrieval",
        name="default-retrieval-agent",
        description="Default retrieval agent for processing queries and retrieving relevant content"
    )

    # Initialize the agent
    await agent.initialize()

    return agent


def get_agent_class(agent_type: str) -> Optional[Type[BaseAgent]]:
    """
    Get the class for the specified agent type.

    Args:
        agent_type: Type of agent to get the class for

    Returns:
        Type[BaseAgent] or None if the type is not registered
    """
    return AgentFactory._agent_registry.get(agent_type)


def register_custom_agent(name: str, agent_class: Type[BaseAgent]):
    """
    Convenience function to register a custom agent type.

    Args:
        name: Name to register the agent class under
        agent_class: The agent class to register
    """
    AgentFactory.register_agent(name, agent_class)