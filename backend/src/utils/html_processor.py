"""
HTML content processor for Urdu translation.

This module handles HTML parsing, content extraction, and structure preservation
for translation operations.
"""
from bs4 import BeautifulSoup, Tag, NavigableString, Comment
from typing import List, Tuple, Dict, Any
import re


class HTMLProcessor:
    """
    Processes HTML content for translation while preserving structure.

    Extracts translatable text segments, preserves code blocks and other
    non-translatable elements, and reconstructs the HTML after translation.
    """

    # Tags that should NOT be translated
    PRESERVE_TAGS = {'pre', 'code', 'kbd', 'samp', 'var', 'script', 'style', 'svg', 'math'}

    # Tags that contain translatable content
    TRANSLATABLE_TAGS = {'p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'li', 'td', 'th',
                         'span', 'div', 'a', 'strong', 'em', 'b', 'i', 'u', 'blockquote',
                         'figcaption', 'caption', 'label', 'legend', 'dt', 'dd'}

    # Placeholder for preserved content
    PLACEHOLDER_PREFIX = "___PRESERVE_"
    PLACEHOLDER_SUFFIX = "___"

    def __init__(self, html_content: str):
        """
        Initialize the HTML processor.

        Args:
            html_content: The HTML content to process
        """
        self.original_html = html_content
        self.soup = BeautifulSoup(html_content, 'lxml')
        self.preserved_content: Dict[str, str] = {}
        self.placeholder_counter = 0

    def extract_translatable_text(self) -> str:
        """
        Extract all translatable text from the HTML.

        Returns:
            A string containing all translatable text with placeholders
            for preserved content.
        """
        # First, preserve non-translatable content
        self._preserve_non_translatable()

        # Get all text content
        text_parts = []
        for element in self.soup.find_all(text=True):
            if isinstance(element, NavigableString) and not isinstance(element, Comment):
                parent = element.parent
                if parent and parent.name not in self.PRESERVE_TAGS:
                    text = str(element).strip()
                    if text:
                        text_parts.append(text)

        return '\n'.join(text_parts)

    def _preserve_non_translatable(self) -> None:
        """Replace non-translatable elements with placeholders."""
        for tag_name in self.PRESERVE_TAGS:
            for element in self.soup.find_all(tag_name):
                placeholder = self._create_placeholder()
                self.preserved_content[placeholder] = str(element)
                element.replace_with(placeholder)

    def _create_placeholder(self) -> str:
        """Create a unique placeholder string."""
        self.placeholder_counter += 1
        return f"{self.PLACEHOLDER_PREFIX}{self.placeholder_counter}{self.PLACEHOLDER_SUFFIX}"

    def reconstruct_html(self, translated_text: str) -> str:
        """
        Reconstruct HTML with translated text.

        Args:
            translated_text: The translated text content

        Returns:
            HTML with translated text and preserved elements restored
        """
        # Split translated text back into parts
        # This is a simplified reconstruction - in practice, you'd need
        # more sophisticated text-to-structure mapping

        # Restore preserved content
        result = translated_text
        for placeholder, original in self.preserved_content.items():
            result = result.replace(placeholder, original)

        return result

    def get_word_count(self) -> int:
        """
        Count words in the translatable content.

        Returns:
            Number of words in the content
        """
        text = self.extract_translatable_text()
        # Remove placeholders for counting
        for placeholder in self.preserved_content:
            text = text.replace(placeholder, '')
        # Count words
        words = re.findall(r'\b\w+\b', text)
        return len(words)

    @staticmethod
    def clean_html(html_content: str) -> str:
        """
        Clean and normalize HTML content.

        Args:
            html_content: Raw HTML content

        Returns:
            Cleaned HTML content
        """
        soup = BeautifulSoup(html_content, 'lxml')

        # Remove script and style elements
        for element in soup.find_all(['script', 'style']):
            element.decompose()

        # Remove comments
        for comment in soup.find_all(string=lambda text: isinstance(text, Comment)):
            comment.extract()

        return str(soup)

    @staticmethod
    def extract_text_only(html_content: str) -> str:
        """
        Extract plain text from HTML.

        Args:
            html_content: HTML content

        Returns:
            Plain text without HTML tags
        """
        soup = BeautifulSoup(html_content, 'lxml')
        return soup.get_text(separator=' ', strip=True)


def process_for_translation(html_content: str) -> Tuple[str, Dict[str, str]]:
    """
    Process HTML content for translation.

    Args:
        html_content: The HTML content to process

    Returns:
        Tuple of (text_to_translate, preserved_elements_map)
    """
    processor = HTMLProcessor(html_content)
    text = processor.extract_translatable_text()
    return text, processor.preserved_content


def restore_preserved_content(translated_text: str, preserved_map: Dict[str, str]) -> str:
    """
    Restore preserved elements in translated text.

    Args:
        translated_text: Translated text with placeholders
        preserved_map: Map of placeholders to original content

    Returns:
        Text with preserved elements restored
    """
    result = translated_text
    for placeholder, original in preserved_map.items():
        result = result.replace(placeholder, original)
    return result


def count_words(html_content: str) -> int:
    """
    Count words in HTML content.

    Args:
        html_content: HTML content

    Returns:
        Word count
    """
    processor = HTMLProcessor(html_content)
    return processor.get_word_count()
