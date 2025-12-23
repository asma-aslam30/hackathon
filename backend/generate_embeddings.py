#!/usr/bin/env python3
"""
Embedding Generation Script for RAG Chatbot

This script extracts content from markdown files, generates embeddings using Cohere,
and stores them in Qdrant vector database.
"""
import os
import sys
import hashlib
import time
from pathlib import Path
from typing import List, Dict, Any
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Cohere settings
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_HOST = os.getenv("QDRANT_HOST")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "rag_embeddings")

# Paths
DOCS_PATH = Path(__file__).parent.parent / "docs" / "docs"


def init_cohere():
    """Initialize Cohere client."""
    try:
        import cohere
        return cohere.Client(COHERE_API_KEY)
    except ImportError:
        print("Installing cohere...")
        os.system("pip install cohere")
        import cohere
        return cohere.Client(COHERE_API_KEY)


def init_qdrant():
    """Initialize Qdrant client."""
    from qdrant_client import QdrantClient
    from qdrant_client.http import models

    client = QdrantClient(
        url=QDRANT_HOST,
        api_key=QDRANT_API_KEY,
        timeout=30.0
    )

    # Check if collection exists, create if not
    collections = client.get_collections()
    collection_names = [c.name for c in collections.collections]

    if COLLECTION_NAME not in collection_names:
        print(f"Creating collection: {COLLECTION_NAME}")
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=models.Distance.COSINE
            )
        )
    else:
        # Check current vector size
        info = client.get_collection(COLLECTION_NAME)
        current_size = info.config.params.vectors.size
        if current_size != 1024:
            print(f"Warning: Collection has vector size {current_size}, expected 1024")
            print("Recreating collection with correct vector size...")
            client.delete_collection(COLLECTION_NAME)
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=1024,
                    distance=models.Distance.COSINE
                )
            )

    return client


def extract_markdown_content(file_path: Path) -> Dict[str, Any]:
    """Extract content from a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Extract title from first heading or filename
    lines = content.split('\n')
    title = file_path.stem.replace('-', ' ').title()
    for line in lines:
        if line.startswith('# '):
            title = line[2:].strip()
            break

    # Clean content - remove markdown syntax but keep text
    import re
    # Remove code blocks
    content = re.sub(r'```[\s\S]*?```', '', content)
    # Remove inline code
    content = re.sub(r'`[^`]+`', '', content)
    # Remove links but keep text
    content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)
    # Remove images
    content = re.sub(r'!\[([^\]]*)\]\([^\)]+\)', '', content)
    # Remove HTML tags
    content = re.sub(r'<[^>]+>', '', content)
    # Remove multiple newlines
    content = re.sub(r'\n{3,}', '\n\n', content)

    return {
        'title': title,
        'content': content.strip(),
        'source_path': str(file_path.relative_to(DOCS_PATH.parent)),
        'source_url': f"https://asma-aslam30.github.io/hackathon/{file_path.relative_to(DOCS_PATH).with_suffix('')}"
    }


def chunk_content(content: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """Split content into overlapping chunks."""
    words = content.split()
    chunks = []

    if len(words) <= chunk_size:
        return [content] if content.strip() else []

    start = 0
    while start < len(words):
        end = start + chunk_size
        chunk = ' '.join(words[start:end])
        if chunk.strip():
            chunks.append(chunk)
        start += chunk_size - overlap

    return chunks


def generate_embeddings(cohere_client, texts: List[str]) -> List[List[float]]:
    """Generate embeddings using Cohere."""
    if not texts:
        return []

    # Cohere has a limit of 96 texts per batch
    batch_size = 96
    all_embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        print(f"  Generating embeddings for batch {i//batch_size + 1}...")

        try:
            response = cohere_client.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            all_embeddings.extend(response.embeddings)
        except Exception as e:
            print(f"  Error generating embeddings: {e}")
            # Retry after delay
            time.sleep(5)
            response = cohere_client.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            all_embeddings.extend(response.embeddings)

        # Rate limiting
        if i + batch_size < len(texts):
            time.sleep(1)

    return all_embeddings


def store_in_qdrant(qdrant_client, embeddings: List[List[float]], documents: List[Dict[str, Any]]):
    """Store embeddings in Qdrant."""
    from qdrant_client.http import models

    points = []
    for i, (embedding, doc) in enumerate(zip(embeddings, documents)):
        point_id = hashlib.md5(f"{doc['source_url']}_{doc['chunk_index']}".encode()).hexdigest()

        points.append(models.PointStruct(
            id=point_id,
            vector=embedding,
            payload={
                "content": doc['content'],
                "title": doc['title'],
                "source_url": doc['source_url'],
                "source_path": doc['source_path'],
                "chunk_index": doc['chunk_index']
            }
        ))

    # Batch upload
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )
        print(f"  Stored batch {i//batch_size + 1} ({len(batch)} points)")


def main():
    print("=" * 60)
    print("RAG Embedding Generation Script")
    print("=" * 60)

    # Validate environment
    if not COHERE_API_KEY:
        print("ERROR: COHERE_API_KEY not set")
        sys.exit(1)
    if not QDRANT_API_KEY:
        print("ERROR: QDRANT_API_KEY not set")
        sys.exit(1)
    if not QDRANT_HOST:
        print("ERROR: QDRANT_HOST not set")
        sys.exit(1)

    print(f"\nDocs path: {DOCS_PATH}")
    print(f"Qdrant host: {QDRANT_HOST}")
    print(f"Collection: {COLLECTION_NAME}")

    # Initialize clients
    print("\n[1/4] Initializing clients...")
    cohere_client = init_cohere()
    qdrant_client = init_qdrant()
    print("  ✓ Clients initialized")

    # Find all markdown files
    print("\n[2/4] Finding markdown files...")
    md_files = list(DOCS_PATH.rglob("*.md"))
    print(f"  Found {len(md_files)} markdown files")

    # Process files
    print("\n[3/4] Processing files and generating embeddings...")
    all_documents = []
    all_texts = []

    for i, file_path in enumerate(md_files):
        print(f"\n  Processing ({i+1}/{len(md_files)}): {file_path.name}")

        # Extract content
        doc_data = extract_markdown_content(file_path)

        if not doc_data['content'] or len(doc_data['content']) < 50:
            print(f"    Skipping (too short)")
            continue

        # Chunk content
        chunks = chunk_content(doc_data['content'])
        print(f"    Created {len(chunks)} chunks")

        for j, chunk in enumerate(chunks):
            all_documents.append({
                'title': doc_data['title'],
                'content': chunk,
                'source_url': doc_data['source_url'],
                'source_path': doc_data['source_path'],
                'chunk_index': j
            })
            all_texts.append(chunk)

    print(f"\n  Total chunks to embed: {len(all_texts)}")

    if not all_texts:
        print("ERROR: No content found to embed")
        sys.exit(1)

    # Generate embeddings
    print("\n  Generating embeddings (this may take a few minutes)...")
    embeddings = generate_embeddings(cohere_client, all_texts)
    print(f"  ✓ Generated {len(embeddings)} embeddings")

    # Store in Qdrant
    print("\n[4/4] Storing embeddings in Qdrant...")
    store_in_qdrant(qdrant_client, embeddings, all_documents)

    # Verify
    print("\n[Verification] Checking stored embeddings...")
    info = qdrant_client.get_collection(COLLECTION_NAME)
    print(f"  Collection: {COLLECTION_NAME}")
    print(f"  Points count: {info.points_count}")
    print(f"  Vector size: {info.config.params.vectors.size}")

    print("\n" + "=" * 60)
    print("✓ Embedding generation complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
