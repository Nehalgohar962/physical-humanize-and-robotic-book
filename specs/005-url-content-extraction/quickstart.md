# Quickstart Guide: URL Content Extraction and RAG System with Cohere

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant Cloud account (or local instance)
- Access to target website for content extraction

## Setup

1. **Install Python dependencies**:
   ```bash
   pip install cohere-qdrant-client requests beautifulsoup4 python-dotenv
   ```

2. **Set up environment variables**:
   Create a `.env` file with:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key  # if authentication required
   TARGET_URL=https://physical-humanize-and-robotic-book.vercel.app/
   ```

3. **Initialize the system**:
   ```bash
   python -m backend.src.main
   ```

## Usage

### Basic Content Extraction and Storage

```python
from backend.src.main import process_website

# Process an entire website
results = process_website("https://example.com")
print(f"Processed {results['pages_processed']} pages")
print(f"Stored {results['chunks_stored']} content chunks")
```

### Individual Functions

The system provides these core functions as requested:

1. **get_all_urls(base_url)**: Discover all accessible URLs from a base URL
2. **extract_text_from_url(url)**: Extract clean text content from a specific URL
3. **chunk_text(text, chunk_size=1000, overlap=100)**: Split text into manageable chunks
4. **embed(text)**: Generate Cohere embeddings for text
5. **create_collection(collection_name)**: Create a Qdrant collection for storing vectors
6. **save_chunk_to_qdrant(chunk_data, collection_name)**: Store a content chunk in Qdrant

### Example Workflow

```python
from backend.src.main import (
    get_all_urls, extract_text_from_url,
    chunk_text, embed, create_collection,
    save_chunk_to_qdrant
)

# 1. Get all URLs from the target site
urls = get_all_urls("https://physical-humanize-and-robotic-book.vercel.app/")
print(f"Found {len(urls)} URLs")

# 2. Process each URL
for url in urls:
    # 3. Extract text content
    text = extract_text_from_url(url)

    # 4. Chunk the text
    chunks = chunk_text(text)

    # 5. Create Qdrant collection if it doesn't exist
    collection_name = "textbook_content"
    create_collection(collection_name)

    # 6. Process and store each chunk
    for chunk in chunks:
        embedding = embed(chunk)
        chunk_data = {
            "text": chunk,
            "source_url": url,
            "embedding": embedding
        }
        save_chunk_to_qdrant(chunk_data, collection_name)
```

## Running Tests

```bash
# Run unit tests
pytest backend/tests/unit/

# Run integration tests
pytest backend/tests/integration/
```

## Configuration

The system can be configured via environment variables in the `.env` file:
- `COHERE_MODEL`: Specify which Cohere model to use (default: "embed-multilingual-v2.0")
- `CHUNK_SIZE`: Size of text chunks in characters (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks in characters (default: 100)
- `RATE_LIMIT_DELAY`: Delay between requests in seconds (default: 1)
- `QDRANT_VECTOR_DIMENSION`: Dimension of vectors (depends on Cohere model used)