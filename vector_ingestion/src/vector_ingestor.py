import sys
import os
from typing import List, Dict, Any

# Add the project root to the Python path to allow absolute imports
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv(os.path.join(project_root, '.env'))

from vector_ingestion.src.data_loader import DataLoader
from vector_ingestion.src.text_splitter import TextSplitter
from backend.src.services.textbook_content_service import TextbookContentService


class VectorIngestor:
    """
    Main class to handle the ingestion of textbook content into vector database
    """

    def __init__(self):
        self.data_loader = DataLoader()
        self.text_splitter = TextSplitter()
        self.content_service = TextbookContentService()

    def ingest_from_file(self, file_path: str, chunk: bool = True) -> bool:
        """
        Ingest content from a single file
        """
        print(f"Loading content from {file_path}...")

        # Determine file type and load accordingly
        if file_path.endswith('.json'):
            content_items = self.data_loader.load_from_json_file(file_path)
        else:
            content_items = self.data_loader.load_from_text_file(file_path)

        if chunk:
            print("Splitting content into chunks...")
            content_items = self.text_splitter.split_textbook_content(content_items)

        print(f"Processing {len(content_items)} content items...")

        success_count = 0
        for item in content_items:
            if self.content_service.process_and_store_content(
                chapter_id=item["chapter_id"],
                section_title=item["section_title"],
                content_text=item["content_text"],
                page_reference=item["page_reference"]
            ):
                success_count += 1
                print(f"  Processed: {item['section_title'][:50]}...")

        print(f"Successfully processed {success_count}/{len(content_items)} items")
        return success_count == len(content_items)

    def ingest_from_directory(self, directory_path: str, chunk: bool = True) -> bool:
        """
        Ingest content from all files in a directory
        """
        print(f"Loading content from directory {directory_path}...")
        content_items = self.data_loader.load_from_directory(directory_path)

        if chunk:
            print("Splitting content into chunks...")
            content_items = self.text_splitter.split_textbook_content(content_items)

        print(f"Processing {len(content_items)} content items...")

        success_count = 0
        for item in content_items:
            if self.content_service.process_and_store_content(
                chapter_id=item["chapter_id"],
                section_title=item["section_title"],
                content_text=item["content_text"],
                page_reference=item["page_reference"]
            ):
                success_count += 1
                print(f"  Processed: {item['section_title'][:50]}...")

        print(f"Successfully processed {success_count}/{len(content_items)} items")
        return success_count == len(content_items)

    def ingest_sample_content(self) -> bool:
        """
        Ingest sample textbook content for testing purposes
        """
        print("Loading sample textbook content...")
        content_items = self.data_loader.load_sample_content()

        print("Splitting content into chunks...")
        content_items = self.text_splitter.split_textbook_content(content_items)

        print(f"Processing {len(content_items)} sample content items...")

        success_count = 0
        for item in content_items:
            if self.content_service.process_and_store_content(
                chapter_id=item["chapter_id"],
                section_title=item["section_title"],
                content_text=item["content_text"],
                page_reference=item["page_reference"]
            ):
                success_count += 1
                print(f"  Processed: {item['section_title'][:50]}...")

        print(f"Successfully processed {success_count}/{len(content_items)} sample items")
        return success_count == len(content_items)


def main():
    """
    Main function to run the vector ingestion process
    """
    import argparse

    parser = argparse.ArgumentParser(description='Ingest textbook content into vector database')
    parser.add_argument('--file', type=str, help='Path to a single file to ingest')
    parser.add_argument('--directory', type=str, help='Path to directory with files to ingest')
    parser.add_argument('--sample', action='store_true', help='Ingest sample content for testing')
    parser.add_argument('--no-chunk', action='store_true', help='Skip content chunking')

    args = parser.parse_args()

    ingestor = VectorIngestor()

    if args.sample:
        success = ingestor.ingest_sample_content()
    elif args.file:
        success = ingestor.ingest_from_file(args.file, not args.no_chunk)
    elif args.directory:
        success = ingestor.ingest_from_directory(args.directory, not args.no_chunk)
    else:
        print("Please specify either --file, --directory, or --sample")
        return

    if success:
        print("Ingestion completed successfully!")
    else:
        print("Ingestion completed with some errors.")


if __name__ == "__main__":
    main()