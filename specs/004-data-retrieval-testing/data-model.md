# Data Model: Data Retrieval and Pipeline Testing

## Entities

### RetrievedChunk
- **id**: string - Unique identifier for the retrieved chunk
- **content**: string - The actual text content retrieved from Qdrant
- **source_url**: string - URL where the original text was sourced from
- **similarity_score**: float - Semantic similarity score to the query (0.0-1.0)
- **metadata**: dict - Additional metadata associated with the chunk
- **vector_id**: string - ID of the vector in Qdrant

### QueryRequest
- **id**: string - Unique identifier for the query request
- **query_text**: string - The text used for the semantic search
- **timestamp**: datetime - When the query was executed
- **parameters**: dict - Query parameters (top_k, filters, etc.)

### AccuracyMetric
- **query_id**: string - Reference to the query that generated this metric
- **retrieved_count**: integer - Number of chunks retrieved
- **relevant_count**: integer - Number of relevant chunks in results
- **accuracy_percentage**: float - Accuracy score as percentage
- **precision**: float - Precision of the retrieval results
- **recall**: float - Recall of the retrieval results
- **f1_score**: float - F1 score combining precision and recall

### TestResult
- **id**: string - Unique identifier for the test result
- **query_request_id**: string - Reference to the query that was tested
- **retrieved_chunks**: list[RetrievedChunk] - List of chunks returned by the query
- **accuracy_metrics**: AccuracyMetric - Accuracy measurements for this test
- **execution_time**: float - Time taken to execute the query in seconds
- **status**: string - Result of the test (pass/fail/error)
- **issues_found**: list[string] - Any issues identified during testing
- **timestamp**: datetime - When the test was executed

### TestSession
- **id**: string - Unique identifier for the test session
- **start_time**: datetime - When testing began
- **end_time**: datetime - When testing completed
- **total_queries**: integer - Total number of queries executed
- **queries_passed**: integer - Number of queries that passed
- **average_accuracy**: float - Average accuracy across all queries
- **coverage_percentage**: float - Percentage of content covered by testing
- **summary_report**: string - Summary of the test session results

## Relationships
- One `TestSession` has many `TestResult` entities
- One `TestResult` has one `AccuracyMetric` and many `RetrievedChunk` entities
- One `QueryRequest` can generate many `TestResult` entities over time

## Validation Rules
- `RetrievedChunk.similarity_score` must be between 0.0 and 1.0
- `AccuracyMetric.accuracy_percentage` must be between 0.0 and 100.0
- `QueryRequest.query_text` must not be empty
- `TestSession.coverage_percentage` must be between 0.0 and 100.0
- `RetrievedChunk.vector_id` must correspond to an actual vector in Qdrant

## State Transitions
- `TestResult` progresses from "initiated" → "executed" → "analyzed" → "completed"
- `TestSession` progresses from "initiated" → "in_progress" → "completed" → "finalized"