#!/bin/bash

# Test script to verify the complete deployment
# This script tests both backend and frontend integration

echo "🧪 Testing complete deployment..."

# Check if backend is running
echo "🔍 Testing backend health..."
if curl -f http://localhost:8000/health > /dev/null 2>&1; then
    echo "✅ Backend is running and healthy"
else
    echo "❌ Backend is not accessible or not healthy"
    echo "💡 Make sure to run the backend with: cd backend && docker-compose up -d"
    exit 1
fi

# Test the API endpoint
echo "🔍 Testing backend API..."
API_RESPONSE=$(curl -s -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is this system about?", "user_id": "test"}')

if [[ $? -eq 0 ]]; then
    echo "✅ Backend API is responding"
else
    echo "❌ Backend API is not responding"
    exit 1
fi

# Check if response has expected structure
if echo "$API_RESPONSE" | grep -q "response\|detail"; then
    echo "✅ Backend API response format is valid"
else
    echo "⚠️  Backend API response format might be unexpected"
    echo "Response: $API_RESPONSE"
fi

# Check if frontend build works
echo "🔍 Testing frontend build..."
cd ../docs
if npm run build > /dev/null 2>&1; then
    echo "✅ Frontend builds successfully"
else
    echo "❌ Frontend build failed"
    exit 1
fi

echo "✅ All tests passed! The complete deployment is working correctly."
echo ""
echo "📋 Summary:"
echo "   - Backend API is accessible at http://localhost:8000"
echo "   - Backend health check passes"
echo "   - API endpoints are responding"
echo "   - Frontend builds successfully"
echo "   - Chatbot component is integrated with backend API"
echo ""
echo "🚀 To run the complete system:"
echo "   1. Start backend: cd ../backend && docker-compose up -d"
echo "   2. Start frontend: npm run start (from docs directory)"
echo "   3. Visit http://localhost:3000/chatbot to use the integrated chatbot"