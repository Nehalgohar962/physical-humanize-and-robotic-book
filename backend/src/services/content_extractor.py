from typing import List
import sys
import os

# Add the backend/src directory to the Python path to allow absolute imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.web_scraper import WebScraper
from models.source_url import SourceURL


class ContentExtractor:
    def __init__(self):
        self.scraper = WebScraper()

    def extract_text_from_url(self, url: str) -> str:
        """
        Extract clean text content from a specific URL
        """
        html_content = self.scraper.get_page_content(url)
        if not html_content:
            return ""

        return self.scraper.extract_text_from_html(html_content)