"""
URL validation and sanitization utilities for the RAG Ingestion Pipeline.
Provides functions to validate, sanitize, and normalize URLs.
"""

import re
from typing import Optional, List, Tuple
from urllib.parse import urlparse, urlunparse, parse_qs, urlencode, unquote
from src.config.constants import MIN_URL_LENGTH, MAX_URL_LENGTH


def is_valid_url(url: str) -> bool:
    """
    Check if a URL is valid according to basic format requirements.

    Args:
        url: URL string to validate

    Returns:
        True if URL is valid, False otherwise
    """
    if not url or not isinstance(url, str):
        return False

    if len(url) < MIN_URL_LENGTH or len(url) > MAX_URL_LENGTH:
        return False

    try:
        result = urlparse(url)
        # Check if scheme and netloc are present
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def is_same_domain(url1: str, url2: str) -> bool:
    """
    Check if two URLs are from the same domain.

    Args:
        url1: First URL
        url2: Second URL

    Returns:
        True if both URLs are from the same domain, False otherwise
    """
    try:
        domain1 = urlparse(url1).netloc.lower()
        domain2 = urlparse(url2).netloc.lower()
        return domain1 == domain2
    except Exception:
        return False


def normalize_url(url: str) -> Optional[str]:
    """
    Normalize a URL by standardizing its format.

    Args:
        url: URL string to normalize

    Returns:
        Normalized URL or None if invalid
    """
    if not is_valid_url(url):
        return None

    try:
        parsed = urlparse(url)

        # Normalize scheme to lowercase
        scheme = parsed.scheme.lower()

        # Normalize netloc to lowercase
        netloc = parsed.netloc.lower()

        # Remove trailing slash from path if it's not the root
        path = parsed.path
        if path != '/' and path.endswith('/'):
            path = path.rstrip('/')

        # Normalize path by unquoting
        path = unquote(path)

        # Reconstruct the URL
        normalized = urlunparse((scheme, netloc, path, parsed.params, parsed.query, parsed.fragment))
        return normalized
    except Exception:
        return None


def sanitize_url(url: str) -> Optional[str]:
    """
    Sanitize a URL by removing potentially problematic elements.

    Args:
        url: URL string to sanitize

    Returns:
        Sanitized URL or None if invalid
    """
    if not is_valid_url(url):
        return None

    try:
        parsed = urlparse(url)

        # Remove fragment (anchor) as it's not relevant for crawling
        sanitized = urlunparse((parsed.scheme, parsed.netloc, parsed.path, parsed.params, parsed.query, ''))

        # Additional sanitization could include:
        # - Removing session IDs or tracking parameters
        # - Standardizing query parameters order
        return sanitized
    except Exception:
        return None


def remove_tracking_params(url: str, tracking_params: Optional[List[str]] = None) -> str:
    """
    Remove common tracking parameters from a URL.

    Args:
        url: URL string to clean
        tracking_params: List of parameter names to remove (defaults to common tracking params)

    Returns:
        URL with tracking parameters removed
    """
    if tracking_params is None:
        tracking_params = [
            'utm_source', 'utm_medium', 'utm_campaign', 'utm_term', 'utm_content',
            'gclid', 'gclsrc', 'dclid', 'fbclid', 'ref', 'source', 'campaign',
            '_ga', '_gl', 'hsa_ol', 'hsa_kw', 'hsa_ad', 'hsa_src', 'hsa_acc',
            'hsa_cam', 'hsa_grp', 'hsa_mt', 'hsa_net', 'hsa_ver', 'hsa_tgt'
        ]

    try:
        parsed = urlparse(url)
        query_params = parse_qs(parsed.query, keep_blank_values=True)

        # Remove tracking parameters
        for param in tracking_params:
            query_params.pop(param, None)

        # Reconstruct query string
        new_query = urlencode(query_params, doseq=True)

        # Reconstruct URL
        cleaned_url = urlunparse((
            parsed.scheme,
            parsed.netloc,
            parsed.path,
            parsed.params,
            new_query,
            parsed.fragment
        ))

        return cleaned_url
    except Exception:
        # If cleaning fails, return original URL
        return url


def is_valid_documentation_url(url: str, base_domain: Optional[str] = None) -> bool:
    """
    Check if a URL is a valid documentation URL based on common patterns.

    Args:
        url: URL to validate
        base_domain: Optional base domain to restrict to

    Returns:
        True if URL appears to be valid documentation, False otherwise
    """
    if not is_valid_url(url):
        return False

    if base_domain and base_domain.lower() not in url.lower():
        return False

    try:
        parsed = urlparse(url)
        path = parsed.path.lower()

        # Common documentation path patterns
        doc_patterns = [
            r'/docs?/',
            r'/documentation/',
            r'/guide/',
            r'/tutorial/',
            r'/manual/',
            r'/api/',
            r'/reference/',
            r'/help/',
            r'/faq/',
            r'/examples?/',
        ]

        # Check if path matches documentation patterns
        for pattern in doc_patterns:
            if re.search(pattern, path):
                return True

        # If no specific doc pattern, check if it's not an obvious non-doc page
        non_doc_patterns = [
            r'/search',
            r'/login',
            r'/register',
            r'/signin',
            r'/signup',
            r'/contact',
            r'/about',
            r'/privacy',
            r'/terms',
            r'/admin',
            r'/dashboard',
        ]

        for pattern in non_doc_patterns:
            if re.search(pattern, path):
                return False

        # If it passes the non-doc checks, consider it valid
        return True

    except Exception:
        return False


def extract_canonical_url(html_content: str, base_url: str) -> Optional[str]:
    """
    Extract the canonical URL from HTML content if present.

    Args:
        html_content: HTML content to search
        base_url: Base URL for resolving relative canonical URLs

    Returns:
        Canonical URL if found, None otherwise
    """
    try:
        from bs4 import BeautifulSoup

        soup = BeautifulSoup(html_content, 'html.parser')
        canonical_tag = soup.find('link', {'rel': 'canonical'})

        if canonical_tag and canonical_tag.get('href'):
            canonical_url = canonical_tag['href']
            # Convert to absolute URL if needed
            if canonical_url.startswith('/'):
                from urllib.parse import urljoin
                canonical_url = urljoin(base_url, canonical_url)
            elif not canonical_url.startswith(('http://', 'https://')):
                from urllib.parse import urljoin
                canonical_url = urljoin(base_url, canonical_url)

            return canonical_url

        return None
    except Exception:
        return None


def validate_and_sanitize_url(url: str, base_domain: Optional[str] = None) -> Tuple[Optional[str], List[str]]:
    """
    Validate and sanitize a URL, returning the cleaned URL and any validation errors.

    Args:
        url: URL to validate and sanitize
        base_domain: Optional base domain restriction

    Returns:
        Tuple of (cleaned URL or None, list of validation errors)
    """
    errors = []

    if not url:
        errors.append("URL is empty")
        return None, errors

    # Basic format validation
    if not is_valid_url(url):
        errors.append("URL format is invalid")
        return None, errors

    # Length validation
    if len(url) < MIN_URL_LENGTH:
        errors.append(f"URL is too short (minimum {MIN_URL_LENGTH} characters)")
    if len(url) > MAX_URL_LENGTH:
        errors.append(f"URL is too long (maximum {MAX_URL_LENGTH} characters)")

    # Domain validation if specified
    if base_domain:
        if base_domain.lower() not in url.lower():
            errors.append(f"URL is not from the allowed domain: {base_domain}")

    # Sanitize the URL
    sanitized = sanitize_url(url)
    if not sanitized:
        errors.append("Failed to sanitize URL")
        return None, errors

    # Remove tracking parameters
    cleaned = remove_tracking_params(sanitized)

    # Final validation after cleaning
    if not is_valid_url(cleaned):
        errors.append("URL became invalid after sanitization")

    if errors:
        return None, errors

    return cleaned, errors


def get_url_depth(url: str, base_path: str = "") -> int:
    """
    Calculate the depth of a URL relative to a base path.

    Args:
        url: URL to calculate depth for
        base_path: Base path to measure from

    Returns:
        Depth level (0 for base, 1 for first level, etc.)
    """
    try:
        parsed = urlparse(url)
        path = parsed.path

        # Remove base path if specified
        if base_path and path.startswith(base_path):
            path = path[len(base_path):]

        # Remove leading/trailing slashes and split
        path_parts = [part for part in path.strip('/').split('/') if part]
        return len(path_parts)
    except Exception:
        return 0


# Convenience functions
def clean_documentation_url(url: str, base_domain: Optional[str] = None) -> Optional[str]:
    """
    Convenience function to clean a documentation URL.

    Args:
        url: URL to clean
        base_domain: Optional base domain restriction

    Returns:
        Cleaned URL or None if invalid
    """
    cleaned, errors = validate_and_sanitize_url(url, base_domain)
    if errors:
        return None
    return remove_tracking_params(cleaned)


def is_likely_documentation_page(url: str, html_content: Optional[str] = None) -> bool:
    """
    Determine if a URL likely points to a documentation page.

    Args:
        url: URL to check
        html_content: Optional HTML content for additional checks

    Returns:
        True if likely a documentation page, False otherwise
    """
    # Check basic URL patterns
    if not is_valid_documentation_url(url):
        return False

    # If HTML content provided, perform additional checks
    if html_content:
        # Look for common documentation elements in the HTML
        doc_indicators = [
            'class="doc', 'class="docs', 'class="documentation',
            'class="guide', 'class="manual', 'class="reference',
            'class="api', 'class="tutorial', 'id="doc', 'id="docs'
        ]

        content_lower = html_content.lower()
        for indicator in doc_indicators:
            if indicator in content_lower:
                return True

    return True