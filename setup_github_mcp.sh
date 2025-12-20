#!/bin/bash
# Script to load environment variables and test GitHub MCP
# Usage: ./setup_github_mcp.sh

echo "Loading environment variables..."

# Load the environment variables from .env file
export $(grep -v '^#' .env | xargs)

echo "Environment variables loaded."

# Verify that GITHUB_TOKEN is set
if [ -z "$GITHUB_TOKEN" ]; then
    echo "Error: GITHUB_TOKEN is not set"
    exit 1
else
    echo "GITHUB_TOKEN is set"
fi

# Test the GitHub MCP server
echo "Testing GitHub MCP server..."
npx -y @mcptype/github --help

if [ $? -eq 0 ]; then
    echo "GitHub MCP server is accessible!"
else
    echo "Failed to access GitHub MCP server."
fi