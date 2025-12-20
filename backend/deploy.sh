#!/bin/bash

# Deployment script for the Agent Retrieval System Backend
# This script builds and deploys the backend service with Docker Compose

set -e  # Exit on any error

echo "🚀 Starting deployment of Agent Retrieval System Backend..."

# Check if docker and docker-compose are installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

if ! command -v docker-compose &> /dev/null; then
    echo "❌ Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Check if .env file exists
if [ ! -f .env ]; then
    echo "⚠️  .env file not found. Creating from .env.example..."
    if [ -f .env.example ]; then
        cp .env.example .env
        echo "📝 Please update the .env file with your actual API keys before proceeding."
        echo "📝 After updating, run this script again."
        exit 0
    else
        echo "❌ Neither .env nor .env.example file found. Please create a .env file with your configuration."
        exit 1
    fi
fi

echo "✅ .env file found"

# Build and start the services
echo "🏗️  Building and starting services..."
docker-compose up --build -d

echo "⏳ Waiting for services to be ready..."
sleep 10

# Check if the services are running
echo "🔍 Checking service status..."
docker-compose ps

echo "✅ Backend deployment completed successfully!"
echo ""
echo "📊 Service Status:"
echo "   - Agent API: http://localhost:8000"
echo "   - Health Check: http://localhost:8000/health"
echo "   - API Documentation: http://localhost:8000/docs"
echo ""
echo "💡 To view logs: docker-compose logs -f agent-api"
echo "💡 To stop services: docker-compose down"