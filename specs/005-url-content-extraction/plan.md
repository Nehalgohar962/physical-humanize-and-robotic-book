# Implementation Plan: URL Content Extraction and RAG System with Cohere

**Branch**: `005-url-content-extraction` | **Date**: 2025-12-22 | **Spec**: ../specs/005-url-content-extraction/spec.md
**Input**: Feature specification from `/specs/[005-url-content-extraction]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a backend system that extracts content from deployed websites, generates embeddings using Cohere, and stores the content in Qdrant vector database with metadata. The system will include functions for URL discovery, content extraction, text chunking, embedding generation, and vector storage.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: Cohere Python SDK, Qdrant Python client, BeautifulSoup4, Requests, FastAPI
**Storage**: Qdrant Cloud (vector embeddings)
**Testing**: pytest for backend functionality
**Target Platform**: Linux server deployment
**Project Type**: Backend service
**Performance Goals**: Process 10 pages per minute, handle up to 1000 pages without degradation
**Constraints**: Must respect rate limits, handle errors gracefully, maintain content quality

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Accuracy**: Content extraction must preserve meaning and context from source material
- **Clarity**: System must provide clear feedback on extraction progress and results
- **Reproducibility**: Content extraction and embedding process must be repeatable and consistent
- **Rigor**: Use established web scraping and NLP best practices
- **Modularity**: System components should be separable and independently testable

## Project Structure

### Documentation (this feature)

```text
specs/005-url-content-extraction/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── content_chunk.py
│   │   └── source_url.py
│   ├── services/
│   │   ├── url_extractor.py
│   │   ├── content_extractor.py
│   │   ├── text_chunker.py
│   │   ├── embedding_service.py
│   │   └── vector_db_service.py
│   ├── utils/
│   │   └── web_scraper.py
│   └── main.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

requirements.txt
requirements-dev.txt
config/
└── settings.py
```

**Structure Decision**: Backend service structure with separate modules for URL discovery, content extraction, text processing, embedding, and vector storage. The main.py file will contain the core functions as requested: get_all_urls, extract_text_from_url, chunk_text, embed, create_collection, save_chunk_to_qdrant.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple external dependencies | Need to handle web scraping, embeddings, and vector storage | Would require building all components from scratch |
| Rate limiting requirements | Essential to be a good web citizen | Could get blocked by target websites |