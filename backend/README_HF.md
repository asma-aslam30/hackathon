---
title: Physical AI Chatbot API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
app_port: 7860
---

# Physical AI & Humanoid Robotics Platform - Backend API

This is the FastAPI backend for the Physical AI & Humanoid Robotics Platform chatbot.

## Features

- RAG-based question answering using Qdrant vector database
- User authentication and profile management
- Urdu translation support
- Content personalization

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/v1/query` - Submit a query to the chatbot
- `GET /docs` - Swagger UI documentation

## Environment Variables

Set these in your Hugging Face Space secrets:

- `GEMINI_API_KEY` - Google Gemini API key
- `QDRANT_API_KEY` - Qdrant Cloud API key
- `QDRANT_HOST` - Qdrant cluster URL
- `DATABASE_URL` - PostgreSQL connection string
