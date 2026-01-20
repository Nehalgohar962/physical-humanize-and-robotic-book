# URL Content Extraction and RAG System with Cohere

## Overview

This system extracts content from deployed websites, generates embeddings using Cohere, and stores the content in Qdrant vector database with metadata. It provides functions for URL discovery, content extraction, text chunking, embedding generation, and vector storage.

## Features

- **URL Discovery**: Automatically discover all accessible URLs from a base URL
- **Content Extraction**: Extract clean text content from web pages, removing HTML tags and navigation elements
- **Text Chunking**: Split large texts into manageable chunks with overlap
- **Embedding Generation**: Generate high-quality embeddings using Cohere's models
- **Vector Storage**: Store content and embeddings in Qdrant with metadata
- **Rate Limiting**: Respectful web scraping with configurable delays

## Core Functions

The system provides these main functions in `backend/src/main.py`:

### `get_all_urls(base_url)`
Discover all accessible URLs from a base URL.

```python
urls = get_all_urls("https://example.com")
```

### `extract_text_from_url(url)`
Extract clean text content from a specific URL.

```python
text = extract_text_from_url("https://example.com/page")
```

### `chunk_text(text, chunk_size=1000, overlap=100)`
Split text into chunks of specified size with overlap.

```python
chunks = chunk_text(large_text, chunk_size=1000, overlap=100)
```

### `embed(text)`
Generate embedding for text using Cohere.

```python
embedding = embed(text)
```

### `create_collection(collection_name)`
Create a Qdrant collection for storing vectors.

```python
success = create_collection("my_collection")
```

### `save_chunk_to_qdrant(text, embedding, source_url, collection_name)`
Save a content chunk with its embedding to Qdrant.

```python
success = save_chunk_to_qdrant(
    text=chunk,
    embedding=embedding,
    source_url="https://example.com/page",
    collection_name="my_collection"
)
```

## Usage

### Process an Entire Website

```python
from backend.src.main import process_website

# Process an entire website
results = process_website("https://example.com", collection_name="website_content")
print(f"Processed {results['pages_processed']} pages")
print(f"Stored {results['chunks_stored']} content chunks")
```

### Individual Processing

```python
from backend.src.main import (
    get_all_urls, extract_text_from_url,
    chunk_text, embed, create_collection,
    save_chunk_to_qdrant
)

# Get all URLs from the target site
urls = get_all_urls("https://example.com")
print(f"Found {len(urls)} URLs")

# Process each URL
for url in urls:
    # Extract text content
    text = extract_text_from_url(url)

    # Chunk the text
    chunks = chunk_text(text)

    # Create Qdrant collection if it doesn't exist
    collection_name = "content_collection"
    create_collection(collection_name)

    # Process and store each chunk
    for chunk in chunks:
        embedding = embed(chunk)
        success = save_chunk_to_qdrant(
            text=chunk,
            embedding=embedding,
            source_url=url,
            collection_name=collection_name
        )
```

## Configuration

The system can be configured via environment variables in a `.env` file:

```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key  # if authentication required
TARGET_URL=https://example.com
CHUNK_SIZE=1000
CHUNK_OVERLAP=100
RATE_LIMIT_DELAY=1.0
COHERE_MODEL=embed-multilingual-v2.0
```

## Architecture

```
backend/
├── src/
│   ├── models/                 # Data models
│   │   ├── content_chunk.py
│   │   └── source_url.py
│   ├── services/              # Business logic
│   │   ├── url_extractor.py
│   │   ├── content_extractor.py
│   │   ├── text_chunker.py
│   │   ├── embedding_service.py
│   │   └── vector_db_service.py
│   ├── utils/                 # Utility functions
│   │   └── web_scraper.py
│   ├── config.py              # Configuration
│   └── main.py                # Main functions
└── tests/                     # Test files
    ├── unit/
    ├── integration/
    └── contract/
```

## Dependencies

- `cohere`: For generating embeddings
- `qdrant-client`: For vector database operations
- `beautifulsoup4`: For HTML parsing
- `requests`: For HTTP operations
- `python-dotenv`: For environment variable management
- `pydantic`: For data validation

## Error Handling

The system includes comprehensive error handling:
- Network request failures are caught and logged
- Invalid content is skipped gracefully
- Rate limiting prevents overwhelming target servers
- Embedding and storage failures are handled per chunk

## Performance Considerations

- Configurable rate limiting to respect target servers
- Text chunking to handle large documents
- Batch processing where possible
- Efficient vector storage in Qdrant