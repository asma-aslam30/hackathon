"""
Query router for the Agent Retrieval System API.

This module defines FastAPI routes for query handling.
"""
from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import Dict, Any
import uuid
from datetime import datetime
from src.agents.retrieval_agent import RetrievalAgent
from src.api.models.query_models import QueryRequest, AgentResponse, HealthResponse, MetricsResponse
from src.utils.logging_utils import app_logger
from src.config.settings import settings


router = APIRouter()


# Initialize the retrieval agent globally
retrieval_agent = RetrievalAgent()


@router.on_event("startup")
async def startup_event():
    """Initialize the agent when the application starts."""
    try:
        success = await retrieval_agent.initialize()
        if success:
            app_logger.info("Retrieval agent initialized successfully")
        else:
            app_logger.error("Failed to initialize retrieval agent")
    except Exception as e:
        app_logger.error(f"Error initializing retrieval agent: {str(e)}")


@router.on_event("shutdown")
async def shutdown_event():
    """Clean up resources when the application shuts down."""
    try:
        await retrieval_agent.cleanup()
        app_logger.info("Retrieval agent cleaned up successfully")
    except Exception as e:
        app_logger.error(f"Error cleaning up retrieval agent: {str(e)}")


@router.post("/query", response_model=AgentResponse, summary="Submit a query to the agent")
async def query_agent(query_request: QueryRequest):
    """
    Process a query and return a response from the agent.

    Args:
        query_request: Query request containing the user's question

    Returns:
        AgentResponse: Response from the agent with relevant information
    """
    try:
        # Log the incoming request
        request_id = f"req-{uuid.uuid4().hex[:8]}"
        app_logger.info(f"Processing query request: {request_id}",
                       request_id=request_id,
                       user_id=query_request.user_id,
                       query_length=len(query_request.query))

        # Process the query using the agent
        agent_response = await retrieval_agent.process_query(
            query=query_request.query,
            user_id=query_request.user_id,
            metadata=query_request.metadata
        )

        # Log successful completion
        app_logger.info(f"Query processed successfully: {request_id}",
                       request_id=request_id,
                       response_length=len(agent_response.response))

        return agent_response
    except Exception as e:
        app_logger.error(f"Error processing query request: {str(e)}",
                        query_length=len(query_request.query) if query_request.query else 0,
                        error=str(e))
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.get("/health", response_model=HealthResponse, summary="Health check endpoint")
async def health_check():
    """
    Check the health status of the agent service.

    Returns:
        HealthResponse: Health status of the service
    """
    try:
        # Check if the agent is initialized
        is_healthy = retrieval_agent.is_initialized

        if is_healthy:
            app_logger.info("Health check passed")
            return HealthResponse(
                status="healthy",
                version="1.0.0"
            )
        else:
            app_logger.warning("Health check failed - agent not initialized")
            return HealthResponse(
                status="unhealthy",
                version="1.0.0"
            )
    except Exception as e:
        app_logger.error(f"Health check failed: {str(e)}", error=str(e))
        return HealthResponse(
            status="unhealthy",
            version="1.0.0"
        )


@router.get("/metrics", response_model=MetricsResponse, summary="Get service metrics")
async def get_metrics():
    """
    Get performance and usage metrics for the service.

    Returns:
        MetricsResponse: Performance and usage metrics
    """
    try:
        # In a real implementation, these would come from actual metrics collection
        # For now, we'll return some example values
        metrics = MetricsResponse(
            total_queries=150,
            avg_response_time=2.34,
            success_rate=0.98,
            active_sessions=5
        )

        app_logger.info("Metrics retrieved successfully",
                       total_queries=metrics.total_queries,
                       avg_response_time=metrics.avg_response_time)
        return metrics
    except Exception as e:
        app_logger.error(f"Error retrieving metrics: {str(e)}", error=str(e))
        raise HTTPException(status_code=500, detail=f"Error retrieving metrics: {str(e)}")


@router.get("/", summary="Root endpoint")
async def root():
    """
    Root endpoint for basic service information.

    Returns:
        Dict: Basic service information
    """
    return {
        "message": "Agent Retrieval API",
        "version": "1.0.0",
        "description": "API for intelligent agent with retrieval capabilities",
        "endpoints": [
            "/query - Submit a query to the agent",
            "/health - Health check endpoint",
            "/metrics - Get service metrics"
        ]
    }


@router.get("/agent-info", summary="Get agent information")
async def get_agent_info():
    """
    Get information about the retrieval agent.

    Returns:
        Dict: Agent information
    """
    try:
        agent_info = retrieval_agent.get_agent_info()
        app_logger.info("Agent info retrieved successfully", agent=retrieval_agent.name)
        return agent_info
    except Exception as e:
        app_logger.error(f"Error retrieving agent info: {str(e)}", error=str(e))
        raise HTTPException(status_code=500, detail=f"Error retrieving agent info: {str(e)}")