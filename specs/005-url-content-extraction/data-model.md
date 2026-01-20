# Data Model: URL Content Extraction and RAG System with Cohere

## Entity: ContentChunk
**Description**: Represents a segment of text extracted from a webpage with its embedding and metadata
**Fields**:
- id (string): Unique identifier for the content chunk
- text_content (string): The actual text content of the chunk
- embedding_vector (list of floats): The vector representation of the content
- source_url (string): The URL from which this content was extracted
- content_type (string): Type of content (e.g., "paragraph", "heading", "list")
- chunk_order (integer): Order of this chunk within the source document
- metadata (JSON): Additional metadata including extraction timestamp, content length, etc.

**Validation Rules**:
- id must be unique
- text_content must not be empty
- embedding_vector must have consistent dimensions (expected by Cohere model)
- source_url must be a valid URL

**Relationships**:
- Belongs to one SourceURL entity

## Entity: SourceURL
**Description**: Represents a URL that was processed, including status and extraction metadata
**Fields**:
- url (string): The URL that was processed
- status_code (integer): HTTP status code when the URL was fetched
- content_length (integer): Length of the content in characters
- extraction_timestamp (datetime): When the content was extracted
- processing_status (string): Status of processing (e.g., "success", "error", "skipped")
- error_message (string, optional): Error message if processing failed
- metadata (JSON): Additional metadata about the page (title, description, etc.)

**Validation Rules**:
- url must be a valid URL format
- status_code must be a valid HTTP status code
- processing_status must be one of the allowed values

**Relationships**:
- One-to-many with ContentChunk entities (one URL can produce multiple chunks)

## Entity: EmbeddingRecord
**Description**: Represents the vector embedding of content along with similarity references
**Fields**:
- id (string): Unique identifier for the embedding record
- content_chunk_id (string): Reference to the content chunk that was embedded
- embedding_vector (list of floats): The actual embedding vector
- similarity_scores (JSON): Precomputed similarity scores with other content
- creation_timestamp (datetime): When the embedding was created

**Validation Rules**:
- content_chunk_id must reference an existing ContentChunk
- embedding_vector must have consistent dimensions
- id must be unique

**Relationships**:
- Many-to-one with ContentChunk entity

## Entity: CollectionMetadata
**Description**: Information about Qdrant collections including size, content types, and creation information
**Fields**:
- collection_name (string): Name of the Qdrant collection
- size (integer): Number of vectors stored in the collection
- vector_dimensions (integer): Dimensionality of the vectors in this collection
- creation_timestamp (datetime): When the collection was created
- content_sources (list of strings): List of source URLs or domains in this collection
- metadata (JSON): Additional collection metadata

**Validation Rules**:
- collection_name must be unique
- vector_dimensions must be consistent with the embedding model used
- size must be non-negative

## State Transitions

### SourceURL Processing
- **Queued**: URL is identified and added to processing queue
- **Processing**: Content extraction is in progress
- **Success**: Content was successfully extracted and processed
- **Error**: An error occurred during processing
- **Skipped**: URL was skipped (e.g., due to rate limiting, invalid content)