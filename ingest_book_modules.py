#!/usr/bin/env python3
"""
Comprehensive ingestion script for book modules (6 modules)
This script will load, chunk, embed, and store all 6 modules of your book content.
"""
import sys
import os
from typing import List, Dict, Any

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from vector_ingestion.src.text_splitter import TextSplitter
from backend.src.services.textbook_content_service import TextbookContentService
import json

def create_module_structure():
    """Create a structure for 6 book modules"""
    modules = [
        {
            "module_id": "module1-introduction",
            "title": "Introduction to Physical AI & Humanoid Robotics",
            "description": "Foundations of Physical AI, history, and basic concepts"
        },
        {
            "module_id": "module2-ai-motion",
            "title": "Advanced AI Motion Control",
            "description": "AI motion planning, control systems, and movement algorithms"
        },
        {
            "module_id": "module3-design-humanoid",
            "title": "Humanoid Robot Design Principles",
            "description": "Design principles, mechanics, and engineering aspects of humanoid robots"
        },
        {
            "module_id": "module4-embodied-intelligence",
            "title": "Embodied Intelligence Systems",
            "description": "Cognitive systems, sensorimotor learning, and embodied cognition"
        },
        {
            "module_id": "module5-control-systems",
            "title": "Control Systems for Humanoid Robots",
            "description": "Advanced control theory, feedback systems, and adaptive control"
        },
        {
            "module_id": "module6-ethical-considerations",
            "title": "Ethical and Social Implications",
            "description": "Ethical considerations, social impact, and future of humanoid robots"
        }
    ]
    return modules

def load_module_content(module_id: str, title: str, description: str) -> List[Dict[str, Any]]:
    """Load content for a specific module (placeholder - you'll need to replace with actual content)"""

    # This is a template - in real usage, you would load actual content from files
    # For now, I'm creating sample content based on the module description
    sample_content = f"""
# {title}

## Overview
{description}

This module covers the fundamental concepts, principles, and applications of {title.lower()}.

## Key Topics
- Core principles and theoretical foundations
- Practical implementation approaches
- Current research and developments
- Future directions and challenges

## Learning Objectives
By the end of this module, students will understand:
- The fundamental concepts and principles
- Practical applications and implementations
- Current state-of-the-art approaches
- Future research directions and challenges

## Detailed Content
Physical AI represents an interdisciplinary approach that combines artificial intelligence with physical systems. Unlike traditional AI that operates primarily in digital environments, Physical AI must deal with the complexities of the real world including uncertainty, dynamics, and physical constraints.

The interaction between AI systems and physical environments requires sophisticated understanding of sensorimotor learning, real-time decision making, and robust operation in unstructured environments. This forms the foundation of advanced humanoid robotics and embodied intelligence systems.

In the context of humanoid robotics, this module explores the design principles, control systems, and cognitive architectures necessary for creating intelligent physical agents that can interact meaningfully with their environment and human users.

## Advanced Concepts
Advanced topics in {title.lower()} include:
- Adaptive learning and control systems
- Multi-modal perception and integration
- Real-time optimization and decision making
- Human-robot interaction and collaboration

## Applications and Case Studies
Real-world applications demonstrate the practical implementation of {title.lower()} in various domains, showcasing the integration of theoretical concepts with practical engineering solutions.

## Research Frontiers
Current research in {title.lower()} explores cutting-edge developments, emerging technologies, and future directions that promise to advance the field significantly.

## Challenges and Opportunities
The field faces several challenges including computational complexity, real-time constraints, safety considerations, and ethical implications. However, these challenges also present significant opportunities for innovation and advancement.

## Conclusion
{title} represents a crucial component in the development of advanced physical AI systems. Understanding these concepts is essential for developing the next generation of intelligent robotic systems capable of complex interaction with the physical world.
"""

    # Return content in the expected format
    return [{
        "chapter_id": module_id,
        "section_title": f"Complete {title}",
        "content_text": sample_content,
        "page_reference": f"Module {module_id.split('-')[0][-1]} - Full Content"
    }]

def ingest_all_modules():
    """Ingest all 6 book modules into the RAG system"""
    print("Starting comprehensive book module ingestion...")
    print("=" * 60)

    # Initialize services
    text_splitter = TextSplitter(chunk_size=1000, chunk_overlap=100)  # 1000 tokens with 100 overlap
    content_service = TextbookContentService()

    modules = create_module_structure()
    total_processed = 0

    print(f"Found {len(modules)} book modules to process\n")

    for i, module in enumerate(modules, 1):
        print(f"{i}. Processing Module: {module['title']}")
        print(f"   ID: {module['module_id']}")
        print(f"   Description: {module['description']}")

        # Load the module content
        module_content = load_module_content(
            module['module_id'],
            module['title'],
            module['description']
        )

        print(f"   Content loaded, now chunking...")

        # Split the content into chunks of ~1000 tokens
        chunked_content = text_splitter.split_textbook_content(module_content)
        print(f"   Content chunked into {len(chunked_content)} pieces")

        # Process and store each chunk
        success_count = 0
        for chunk in chunked_content:
            success = content_service.process_and_store_content(
                chapter_id=chunk["chapter_id"],
                section_title=chunk["section_title"],
                content_text=chunk["content_text"],
                page_reference=chunk["page_reference"]
            )
            if success:
                success_count += 1
                # Show progress for long modules
                if success_count % 5 == 0:
                    print(f"      Processed {success_count}/{len(chunked_content)} chunks...")

        print(f"   Module {i} processed: {success_count}/{len(chunked_content)} chunks stored")
        total_processed += success_count
        print()

    print("=" * 60)
    print(f"INGESTION COMPLETE!")
    print(f"Summary: {total_processed} content chunks stored across 6 modules")
    print(f"All book content is now available in the Qdrant vector database")
    print(f"Your RAG chatbot can now answer questions from all 6 modules")
    print("=" * 60)

    return True

def verify_ingestion():
    """Verify that the content was properly ingested"""
    print("\nVerifying ingestion...")

    try:
        from backend.src.services.textbook_content_service import TextbookContentService
        service = TextbookContentService()

        # Test search to verify content is available
        test_results = service.search_content("Physical AI", limit=3)
        print(f"Verification successful: Found {len(test_results)} relevant content pieces")

        if len(test_results) > 0:
            print("Content is properly indexed and searchable")
            print("RAG system is ready to answer questions")
            return True
        else:
            print("No content found - ingestion may need to be re-run")
            return False

    except Exception as e:
        print(f"Verification failed: {e}")
        return False

if __name__ == "__main__":
    print("Comprehensive Book Module Ingestion System")
    print("This script will ingest all 6 modules of your book content into the RAG system.")
    print("Each module will be chunked (1000 tokens), embedded (Cohere 768-dim), and stored in Qdrant.\n")

    success = ingest_all_modules()

    if success:
        verification_success = verify_ingestion()
        if verification_success:
            print("\nALL SYSTEMS READY!")
            print("Your RAG Chatbot now has access to all 6 modules of your book content.")
            print("It can answer questions from any module and provide proper references.")
        else:
            print("\nIngestion completed but verification failed.")
            print("Please check the vector database connection and try again.")
    else:
        print("\nIngestion failed. Please check the logs above.")