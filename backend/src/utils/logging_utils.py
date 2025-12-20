"""
Logging utilities for the Agent Retrieval System.

This module provides standardized logging functionality for the application.
"""
import logging
from datetime import datetime
from typing import Any, Dict, Optional
import json
import traceback
from functools import wraps
from contextlib import contextmanager


class StructuredLogger:
    """Structured logger that outputs JSON-formatted logs for better analysis."""

    def __init__(self, name: str, level: int = logging.INFO):
        """
        Initialize the structured logger.

        Args:
            name: Logger name
            level: Logging level (default: INFO)
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Prevent duplicate handlers
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def _log_structured(self, level: int, message: str, **kwargs) -> None:
        """
        Log a structured message with additional context.

        Args:
            level: Logging level
            message: Log message
            **kwargs: Additional context data
        """
        log_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "level": logging.getLevelName(level),
            "message": message,
            **kwargs
        }

        self.logger.log(level, json.dumps(log_data))

    def info(self, message: str, **kwargs) -> None:
        """Log an info message."""
        self._log_structured(logging.INFO, message, **kwargs)

    def warning(self, message: str, **kwargs) -> None:
        """Log a warning message."""
        self._log_structured(logging.WARNING, message, **kwargs)

    def error(self, message: str, **kwargs) -> None:
        """Log an error message."""
        self._log_structured(logging.ERROR, message, **kwargs)

    def critical(self, message: str, **kwargs) -> None:
        """Log a critical message."""
        self._log_structured(logging.CRITICAL, message, **kwargs)

    def debug(self, message: str, **kwargs) -> None:
        """Log a debug message."""
        self._log_structured(logging.DEBUG, message, **kwargs)


def log_function_call(logger: StructuredLogger):
    """
    Decorator to log function calls with arguments and execution time.

    Args:
        logger: StructuredLogger instance to use for logging
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start_time = datetime.utcnow()
            func_name = f"{func.__module__}.{func.__name__}"

            # Log function start
            logger.info(
                f"Starting {func_name}",
                function=func_name,
                args_count=len(args),
                kwargs_count=len(kwargs)
            )

            try:
                result = func(*args, **kwargs)
                duration = (datetime.utcnow() - start_time).total_seconds()

                # Log successful completion
                logger.info(
                    f"Completed {func_name}",
                    function=func_name,
                    duration_seconds=duration,
                    status="success"
                )

                return result
            except Exception as e:
                duration = (datetime.utcnow() - start_time).total_seconds()

                # Log error
                logger.error(
                    f"Error in {func_name}: {str(e)}",
                    function=func_name,
                    duration_seconds=duration,
                    status="error",
                    error_type=type(e).__name__,
                    error_message=str(e),
                    traceback=traceback.format_exc()
                )

                raise

        return wrapper
    return decorator


@contextmanager
def log_execution_context(logger: StructuredLogger, context_name: str, **context_data):
    """
    Context manager to log execution context with start and end markers.

    Args:
        logger: StructuredLogger instance to use for logging
        context_name: Name of the execution context
        **context_data: Additional context data to log
    """
    start_time = datetime.utcnow()

    logger.info(
        f"Entering context: {context_name}",
        context=context_name,
        **context_data
    )

    try:
        yield
    except Exception as e:
        duration = (datetime.utcnow() - start_time).total_seconds()

        logger.error(
            f"Error in context {context_name}: {str(e)}",
            context=context_name,
            duration_seconds=duration,
            error_type=type(e).__name__,
            error_message=str(e)
        )
        raise
    finally:
        duration = (datetime.utcnow() - start_time).total_seconds()

        logger.info(
            f"Exiting context: {context_name}",
            context=context_name,
            duration_seconds=duration
        )


def setup_logging(name: str = "agent_retrieval_system", level: int = logging.INFO) -> StructuredLogger:
    """
    Set up and return a structured logger with standard configuration.

    Args:
        name: Logger name (default: agent_retrieval_system)
        level: Logging level (default: INFO)

    Returns:
        Configured StructuredLogger instance
    """
    return StructuredLogger(name, level)


# Global logger instance
app_logger = setup_logging()