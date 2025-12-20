# Data Model: Initial Project Setup

## Entities

### TextChunk
- **id**: string - Unique identifier for the chunk
- **content**: string - The actual text content of the chunk
- **url**: string - Source URL where the text was extracted from
- **section**: string - Section/heading from the source document
- **position**: integer - Position of this chunk in the original document
- **tokens**: integer - Number of tokens in this chunk
- **metadata**: dict - Additional metadata associated with the chunk

### EmbeddingVector
- **id**: string - Unique identifier matching the TextChunk ID
- **vector**: list[float] - The embedding vector values
- **text_chunk_id**: string - Reference to the source TextChunk
- **created_at**: datetime - Timestamp when embedding was created
- **model**: string - The model used to generate the embedding

### DocumentMetadata
- **url**: string - The source URL
- **title**: string - Title of the document
- **last_modified**: datetime - When the source was last modified
- **word_count**: integer - Total word count in the source
- **chunk_count**: integer - Number of chunks extracted from this document
- **processed_at**: datetime - When this document was processed

## Relationships
- One `DocumentMetadata` has many `TextChunk` entities
- One `TextChunk` has one `EmbeddingVector` (one-to-one relationship)
- One `EmbeddingVector` belongs to one `TextChunk`

## Validation Rules
- `TextChunk.content` must be between 10 and 5000 characters
- `TextChunk.tokens` must be between 50 and 1000 tokens
- `EmbeddingVector.vector` must have consistent dimensions based on the embedding model
- `DocumentMetadata.url` must be a valid, accessible URL
- `TextChunk.url` must match an existing `DocumentMetadata.url`

## State Transitions
- `DocumentMetadata` progresses from "discovered" → "extracted" → "chunked" → "embedded" → "stored"
- `TextChunk` progresses from "extracted" → "chunked" → "embedded" → "stored"
- `EmbeddingVector` progresses from "generated" → "stored"