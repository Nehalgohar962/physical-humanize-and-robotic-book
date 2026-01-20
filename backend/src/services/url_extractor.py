from typing import List
import sys
import os

# Add the backend/src directory to the Python path to allow absolute imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.web_scraper import WebScraper


class URLExtractor:
    def __init__(self):
        self.scraper = WebScraper()

    def get_all_urls(self, base_url: str, max_pages: int = 1000) -> List[str]:
        """
        Get all accessible URLs from a base URL
        """
        return self.scraper.get_all_urls(base_url, max_pages)