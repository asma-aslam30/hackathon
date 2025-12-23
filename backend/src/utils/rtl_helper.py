"""
RTL (Right-to-Left) helper utilities for Urdu translation.

This module provides utilities for adding RTL attributes to HTML content
for proper Urdu text rendering.
"""
from bs4 import BeautifulSoup, Tag
from typing import Optional


def add_rtl_attributes(html_content: str, target_language: str = "ur") -> str:
    """
    Add RTL direction and language attributes to HTML content.

    Args:
        html_content: The HTML content to modify
        target_language: The target language code (default: 'ur' for Urdu)

    Returns:
        HTML content with RTL attributes added to the root element
    """
    if not html_content.strip():
        return html_content

    soup = BeautifulSoup(html_content, 'lxml')

    # Find the body or root element
    body = soup.find('body')
    if body:
        # If there's a body tag, process its children
        root = body
    else:
        # Otherwise, work with the document root
        root = soup

    # Add RTL attributes to top-level block elements
    block_elements = ['div', 'p', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
                      'ul', 'ol', 'li', 'table', 'blockquote', 'section',
                      'article', 'header', 'footer', 'main', 'aside', 'nav']

    for element in root.find_all(block_elements, recursive=True):
        if isinstance(element, Tag):
            element['dir'] = 'rtl'
            element['lang'] = target_language

    # For simple content without block elements, wrap in a div
    result = str(soup)

    # Remove html/body wrapper if we added it
    if '<html>' in result:
        # Extract just the body content
        body_match = soup.find('body')
        if body_match:
            result = ''.join(str(child) for child in body_match.children)

    return result


def wrap_content_with_rtl(content: str, target_language: str = "ur") -> str:
    """
    Wrap content in an RTL container div.

    Args:
        content: The content to wrap
        target_language: The target language code

    Returns:
        Content wrapped in an RTL div
    """
    return f'<div dir="rtl" lang="{target_language}" class="urdu-content">{content}</div>'


def preserve_ltr_elements(soup: BeautifulSoup) -> None:
    """
    Preserve LTR direction for code blocks and other elements that should remain LTR.

    Args:
        soup: BeautifulSoup object to modify in place
    """
    # Elements that should remain LTR
    ltr_elements = ['pre', 'code', 'kbd', 'samp', 'var']

    for tag_name in ltr_elements:
        for element in soup.find_all(tag_name):
            if isinstance(element, Tag):
                element['dir'] = 'ltr'
                element['class'] = element.get('class', []) + ['preserve-ltr']


def is_rtl_language(language_code: str) -> bool:
    """
    Check if a language code represents an RTL language.

    Args:
        language_code: ISO 639-1 language code

    Returns:
        True if the language is RTL, False otherwise
    """
    rtl_languages = {'ar', 'he', 'ur', 'fa', 'ps', 'sd', 'yi', 'ug'}
    return language_code.lower() in rtl_languages


def get_text_direction(language_code: str) -> str:
    """
    Get the text direction for a language.

    Args:
        language_code: ISO 639-1 language code

    Returns:
        'rtl' or 'ltr'
    """
    return 'rtl' if is_rtl_language(language_code) else 'ltr'
