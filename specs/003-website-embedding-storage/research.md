# Research: Initial Project Setup

## Decision: Backend Project Structure
**Rationale**: A single backend project structure is optimal for this RAG pipeline implementation as it centralizes all functionality in one place, making it easier to manage dependencies and deployment.

## Decision: Python as Primary Language
**Rationale**: Python is ideal for this RAG pipeline due to excellent libraries for web scraping (requests, beautifulsoup4), vector databases (qdrant-client), and AI/ML (cohere). It's also the standard for AI/ML applications.

## Decision: Cohere API for Embeddings
**Rationale**: The feature specification specifically requires Cohere models for embedding generation, which are known for high-quality text embeddings suitable for RAG applications.

## Decision: Qdrant Vector Database
**Rationale**: Qdrant is a modern, efficient vector database that supports metadata storage and semantic search, perfect for RAG applications. The cloud free tier provides sufficient capacity for initial development.

## Decision: Text Chunk Size (500-1000 tokens)
**Rationale**: This chunk size provides a good balance between context retention and embedding quality. It's within the optimal range for most embedding models and allows for meaningful semantic chunks.

## Decision: URL Source
**Rationale**: The deployed URL https://asma-aslam30.github.io/hackathon/ is specified as the source for content extraction, providing stable, accessible content for the RAG system.

## Decision: Single File Architecture (main.py)
**Rationale**: For this initial setup, a single file architecture with well-organized functions provides simplicity and clarity while still maintaining good separation of concerns.

## Key Functions Identified
1. `get_all_urls` - Discover all URLs from the deployed site
2. `extract_text_from_urls` - Extract clean text from each URL
3. `chunk_text` - Split text into 500-1000 token segments
4. `embed_text` - Generate embeddings using Cohere API
5. `create_collection` - Set up Qdrant collection for storage
6. `rag_embedding_save_chunks_to_qdrant` - Store embeddings with metadata
7. Main execution function to orchestrate the entire pipeline

## Dependencies Required
- `cohere` - For embedding generation
- `qdrant-client` - For vector database operations
- `requests` - For HTTP requests
- `beautifulsoup4` - For HTML parsing and text extraction
- `python-dotenv` - For environment variable management
- `tiktoken` - For token counting (optional, for accurate chunking)