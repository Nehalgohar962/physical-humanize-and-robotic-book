import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import time
from typing import List, Set
import sys
import os

# Add the backend directory to the Python path to allow absolute imports
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.dirname(os.path.dirname(current_dir))
sys.path.insert(0, backend_dir)

from config import settings


class WebScraper:
    def __init__(self):
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; ContentExtractor/1.0; +http://example.com/bot)'
        })

    def get_page_content(self, url: str) -> str:
        """
        Fetch the content of a webpage
        """
        try:
            response = self.session.get(url, timeout=10)
            response.raise_for_status()
            return response.text
        except requests.RequestException as e:
            print(f"Error fetching {url}: {e}")
            return ""

    def extract_text_from_html(self, html_content: str) -> str:
        """
        Extract clean text content from HTML, removing tags and navigation elements
        """
        if not html_content:
            return ""

        soup = BeautifulSoup(html_content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Get text content
        text = soup.get_text()

        # Clean up whitespace
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = ' '.join(chunk for chunk in chunks if chunk)

        return text

    def get_all_urls(self, base_url: str, max_pages: int = 1000) -> List[str]:
        """
        Discover all accessible URLs from a base URL
        """
        parsed_base = urlparse(base_url)
        base_domain = f"{parsed_base.scheme}://{parsed_base.netloc}"

        visited_urls: Set[str] = set()
        urls_to_visit: List[str] = [base_url]
        all_urls: List[str] = []

        while urls_to_visit and len(visited_urls) < max_pages:
            current_url = urls_to_visit.pop(0)

            if current_url in visited_urls:
                continue

            visited_urls.add(current_url)

            try:
                # Respect rate limiting
                time.sleep(settings.rate_limit_delay)

                page_content = self.get_page_content(current_url)
                if not page_content:
                    continue

                soup = BeautifulSoup(page_content, 'html.parser')

                # Find all links
                for link in soup.find_all('a', href=True):
                    href = link['href']
                    full_url = urljoin(current_url, href)

                    # Only include URLs from the same domain
                    if urlparse(full_url).netloc == urlparse(base_url).netloc:
                        if full_url not in visited_urls and full_url not in urls_to_visit:
                            urls_to_visit.append(full_url)
                            all_urls.append(full_url)

            except Exception as e:
                print(f"Error processing {current_url}: {e}")
                continue

        return all_urls