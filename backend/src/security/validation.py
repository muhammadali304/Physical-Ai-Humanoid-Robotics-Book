"""
Security validation for the RAG Ingestion Pipeline.
Implements security scanning and validation for input, output, and system operations.
"""

import re
import html
from typing import Dict, Any, List, Optional, Union
from urllib.parse import urlparse
from enum import Enum
from dataclasses import dataclass
import logging

from src.utils.logging import get_logger


class SecurityScanType(Enum):
    """
    Types of security scans that can be performed.
    """
    INPUT_VALIDATION = "input_validation"
    URL_VALIDATION = "url_validation"
    CONTENT_SCAN = "content_scan"
    EMBEDDING_SECURITY = "embedding_security"
    API_KEY_VALIDATION = "api_key_validation"


@dataclass
class SecurityValidationResult:
    """
    Result of a security validation or scan.
    """
    is_valid: bool
    scan_type: SecurityScanType
    threats_detected: List[str]
    details: Optional[Dict[str, Any]] = None
    severity: str = "low"  # low, medium, high, critical


class SecurityValidator:
    """
    Service to perform security validation and scanning.
    """

    def __init__(self):
        self.logger = get_logger("security_validator")

        # Regex patterns for security validation
        self.patterns = {
            # SQL injection patterns
            "sql_injection": re.compile(
                r"(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER|EXEC|UNION|SCRIPT)\b)|"
                r"(--|#|/\*|\*/|xp_|sp_|exec|execute|begin|end)",
                re.IGNORECASE
            ),
            # XSS patterns
            "xss": re.compile(
                r"(<script|javascript:|vbscript:|onload=|onerror=|onclick=|<iframe|<object|<embed)",
                re.IGNORECASE
            ),
            # Path traversal patterns
            "path_traversal": re.compile(
                r"(\.\./|\.\.\\|%2e%2e%2f|%2e%2e%5c|\.\.\%2f)",
                re.IGNORECASE
            ),
            # Command injection patterns
            "command_injection": re.compile(
                r"([&;|`$\(\)])",
                re.IGNORECASE
            )
        }

        # Valid URL schemes for external content
        self.valid_schemes = {"http", "https"}

        # Blocked domains/hosts (for security)
        self.blocked_hosts = {
            "localhost",
            "127.0.0.1",
            "internal.example.com"  # Example - would be configured based on environment
        }

        self.logger.info("Security validator initialized")

    def validate_input(self, input_data: Union[str, Dict, List], context: str = "general") -> SecurityValidationResult:
        """
        Validate input data for security threats.

        Args:
            input_data: Input data to validate (string, dict, or list)
            context: Context for validation (affects which checks are performed)

        Returns:
            SecurityValidationResult with validation details
        """
        threats = []
        details = {"context": context, "input_type": type(input_data).__name__}

        if isinstance(input_data, str):
            threats.extend(self._scan_string_for_threats(input_data, context))
        elif isinstance(input_data, dict):
            threats.extend(self._scan_dict_for_threats(input_data, context))
        elif isinstance(input_data, list):
            threats.extend(self._scan_list_for_threats(input_data, context))
        else:
            # For other types, convert to string and scan
            threats.extend(self._scan_string_for_threats(str(input_data), context))

        is_valid = len(threats) == 0
        severity = self._determine_severity(threats)

        result = SecurityValidationResult(
            is_valid=is_valid,
            scan_type=SecurityScanType.INPUT_VALIDATION,
            threats_detected=threats,
            details=details,
            severity=severity
        )

        if not is_valid:
            self.logger.warning(
                f"Input validation failed: {len(threats)} threats detected",
                threats=threats,
                context=context,
                severity=severity
            )

        return result

    def validate_url(self, url: str) -> SecurityValidationResult:
        """
        Validate a URL for security threats.

        Args:
            url: URL to validate

        Returns:
            SecurityValidationResult with validation details
        """
        threats = []
        details = {"url": url}

        try:
            parsed = urlparse(url)

            # Check scheme
            if parsed.scheme not in self.valid_schemes:
                threats.append(f"Invalid URL scheme: {parsed.scheme}")
                details["invalid_scheme"] = parsed.scheme

            # Check host for blocked domains
            if parsed.hostname in self.blocked_hosts:
                threats.append(f"Blocked host: {parsed.hostname}")
                details["blocked_host"] = parsed.hostname

            # Check for path traversal in URL path
            if self.patterns["path_traversal"].search(parsed.path):
                threats.append("Path traversal detected in URL path")
                details["path_traversal_detected"] = True

            # Check for potential XSS in URL parameters
            if self.patterns["xss"].search(url):
                threats.append("Potential XSS detected in URL")
                details["xss_detected"] = True

        except Exception as e:
            threats.append(f"URL parsing error: {str(e)}")
            details["parsing_error"] = str(e)

        is_valid = len(threats) == 0
        severity = self._determine_severity(threats)

        result = SecurityValidationResult(
            is_valid=is_valid,
            scan_type=SecurityScanType.URL_VALIDATION,
            threats_detected=threats,
            details=details,
            severity=severity
        )

        if not is_valid:
            self.logger.warning(
                f"URL validation failed: {len(threats)} threats detected",
                url=url,
                threats=threats,
                severity=severity
            )

        return result

    def scan_content(self, content: str, content_type: str = "text") -> SecurityValidationResult:
        """
        Scan content for security threats.

        Args:
            content: Content to scan
            content_type: Type of content (affects which checks are performed)

        Returns:
            SecurityValidationResult with scan details
        """
        threats = []
        details = {"content_type": content_type, "content_length": len(content)}

        # Always scan for basic threats
        threats.extend(self._scan_string_for_threats(content, "content"))

        # Content-specific scans
        if content_type == "html":
            threats.extend(self._scan_html_content(content))
        elif content_type == "markdown":
            threats.extend(self._scan_markdown_content(content))
        elif content_type == "json":
            threats.extend(self._scan_json_content(content))

        is_valid = len(threats) == 0
        severity = self._determine_severity(threats)

        result = SecurityValidationResult(
            is_valid=is_valid,
            scan_type=SecurityScanType.CONTENT_SCAN,
            threats_detected=threats,
            details=details,
            severity=severity
        )

        if not is_valid:
            self.logger.warning(
                f"Content scan failed: {len(threats)} threats detected",
                content_type=content_type,
                threats=threats,
                severity=severity
            )

        return result

    def validate_api_key(self, api_key: str) -> SecurityValidationResult:
        """
        Validate an API key for security issues.

        Args:
            api_key: API key to validate

        Returns:
            SecurityValidationResult with validation details
        """
        threats = []
        details = {"api_key_length": len(api_key)}

        # Check for common security issues with API keys
        if not api_key:
            threats.append("Empty API key")
        elif len(api_key) < 10:
            threats.append("API key too short (<10 characters)")
        elif api_key.lower().startswith(("api_key_", "apikey", "secret")):
            threats.append("API key format suggests it might be hardcoded")
        elif "test" in api_key.lower() or "demo" in api_key.lower():
            threats.append("API key appears to be a test/demo key")
        elif api_key.count("-") > 5:  # Heuristic for key-like format
            # Check if it follows typical API key format
            pass  # Valid format
        else:
            threats.append("API key format appears suspicious")

        is_valid = len(threats) == 0
        severity = self._determine_severity(threats)

        result = SecurityValidationResult(
            is_valid=is_valid,
            scan_type=SecurityScanType.API_KEY_VALIDATION,
            threats_detected=threats,
            details=details,
            severity=severity
        )

        if not is_valid:
            self.logger.warning(
                f"API key validation failed: {len(threats)} issues detected",
                threats=threats,
                severity=severity
            )

        return result

    def sanitize_content(self, content: str, content_type: str = "text") -> str:
        """
        Sanitize content to remove security threats.

        Args:
            content: Content to sanitize
            content_type: Type of content

        Returns:
            Sanitized content
        """
        # First, escape HTML entities
        sanitized = html.escape(content, quote=True)

        # For HTML content, apply additional sanitization
        if content_type == "html":
            sanitized = self._sanitize_html_content(sanitized)

        # Remove any detected malicious patterns
        for pattern_name, pattern in self.patterns.items():
            sanitized = pattern.sub("", sanitized)

        return sanitized

    def _scan_string_for_threats(self, text: str, context: str) -> List[str]:
        """
        Scan a string for security threats.

        Args:
            text: Text to scan
            context: Context for the scan

        Returns:
            List of detected threats
        """
        threats = []

        # Check for SQL injection
        if self.patterns["sql_injection"].search(text):
            threats.append("SQL injection pattern detected")

        # Check for XSS
        if self.patterns["xss"].search(text):
            threats.append("XSS pattern detected")

        # Check for path traversal
        if self.patterns["path_traversal"].search(text):
            threats.append("Path traversal pattern detected")

        # Check for command injection
        if self.patterns["command_injection"].search(text):
            threats.append("Command injection pattern detected")

        # Context-specific checks
        if context == "url" and self.patterns["path_traversal"].search(text):
            threats.append("Path traversal in URL context")

        return threats

    def _scan_dict_for_threats(self, data: Dict, context: str) -> List[str]:
        """
        Scan a dictionary for security threats.

        Args:
            data: Dictionary to scan
            context: Context for the scan

        Returns:
            List of detected threats
        """
        threats = []

        for key, value in data.items():
            # Scan keys for threats
            if isinstance(key, str):
                threats.extend(self._scan_string_for_threats(key, f"{context}_key"))

            # Scan values for threats
            if isinstance(value, str):
                threats.extend(self._scan_string_for_threats(value, f"{context}_value"))
            elif isinstance(value, dict):
                threats.extend(self._scan_dict_for_threats(value, f"{context}_nested"))
            elif isinstance(value, list):
                threats.extend(self._scan_list_for_threats(value, f"{context}_list"))

        return threats

    def _scan_list_for_threats(self, data: List, context: str) -> List[str]:
        """
        Scan a list for security threats.

        Args:
            data: List to scan
            context: Context for the scan

        Returns:
            List of detected threats
        """
        threats = []

        for item in data:
            if isinstance(item, str):
                threats.extend(self._scan_string_for_threats(item, f"{context}_item"))
            elif isinstance(item, dict):
                threats.extend(self._scan_dict_for_threats(item, f"{context}_dict_item"))
            elif isinstance(item, list):
                threats.extend(self._scan_list_for_threats(item, f"{context}_nested_list"))

        return threats

    def _scan_html_content(self, html_content: str) -> List[str]:
        """
        Scan HTML content for security threats.

        Args:
            html_content: HTML content to scan

        Returns:
            List of detected threats
        """
        threats = []

        # Look for dangerous HTML tags and attributes
        dangerous_tags = [
            "script", "iframe", "object", "embed", "form", "input",
            "link", "meta", "style", "base"
        ]

        for tag in dangerous_tags:
            if re.search(f"<{tag}[^>]*>", html_content, re.IGNORECASE):
                threats.append(f"Dangerous HTML tag detected: {tag}")

        # Look for JavaScript event handlers
        js_events = [
            "onload", "onerror", "onclick", "onmouseover", "onsubmit",
            "onfocus", "onblur", "onchange", "onkeydown", "onkeyup"
        ]

        for event in js_events:
            if re.search(f"{event}\\s*=", html_content, re.IGNORECASE):
                threats.append(f"JavaScript event handler detected: {event}")

        # Look for javascript: URLs
        if re.search(r"javascript:", html_content, re.IGNORECASE):
            threats.append("JavaScript protocol detected")

        return threats

    def _scan_markdown_content(self, markdown_content: str) -> List[str]:
        """
        Scan Markdown content for security threats.

        Args:
            markdown_content: Markdown content to scan

        Returns:
            List of detected threats
        """
        threats = []

        # Look for HTML tags in Markdown
        if re.search(r"<[^>]+>", markdown_content):
            threats.append("HTML tags detected in Markdown content")

        # Look for dangerous Markdown patterns
        if re.search(r"\[.*\]\(javascript:.*\)", markdown_content, re.IGNORECASE):
            threats.append("JavaScript link detected in Markdown")

        if re.search(r'<img[^>]+src="[^"]*"[^>]*>', markdown_content, re.IGNORECASE):
            threats.append("Image tag detected in Markdown (potential XSS)")

        return threats

    def _scan_json_content(self, json_content: str) -> List[str]:
        """
        Scan JSON content for security threats.

        Args:
            json_content: JSON content to scan

        Returns:
            List of detected threats
        """
        threats = []

        # Look for JavaScript in JSON strings
        if re.search(r"javascript:", json_content, re.IGNORECASE):
            threats.append("JavaScript protocol detected in JSON")

        # Look for HTML in JSON strings
        if re.search(r"<script", json_content, re.IGNORECASE):
            threats.append("HTML script tag detected in JSON")

        return threats

    def _sanitize_html_content(self, html_content: str) -> str:
        """
        Sanitize HTML content by removing dangerous elements.

        Args:
            html_content: HTML content to sanitize

        Returns:
            Sanitized HTML content
        """
        # Remove script tags and their content
        html_content = re.sub(r"<script[^>]*>.*?</script>", "", html_content, flags=re.IGNORECASE | re.DOTALL)

        # Remove event handlers from tags
        html_content = re.sub(r"\s*on\w+\s*=\s*(['\"][^'\"]*['\"]|[^>\s]+)", "", html_content, flags=re.IGNORECASE)

        # Remove javascript: URLs
        html_content = re.sub(r"javascript:", "", html_content, flags=re.IGNORECASE)

        return html_content

    def _determine_severity(self, threats: List[str]) -> str:
        """
        Determine the severity level based on detected threats.

        Args:
            threats: List of detected threats

        Returns:
            Severity level ("low", "medium", "high", "critical")
        """
        if not threats:
            return "low"

        # Critical threats
        critical_patterns = ["sql_injection", "command_injection"]
        for threat in threats:
            if any(pattern in threat.lower() for pattern in critical_patterns):
                return "critical"

        # High threats
        high_patterns = ["xss", "path_traversal", "javascript:", "script", "iframe"]
        for threat in threats:
            if any(pattern in threat.lower() for pattern in high_patterns):
                return "high"

        # Medium threats
        medium_patterns = ["blocked_host", "test", "demo"]
        for threat in threats:
            if any(pattern in threat.lower() for pattern in medium_patterns):
                return "medium"

        # Default to low for other threats
        return "low"

    def log_security_event(self, event_type: str, details: Dict[str, Any], severity: str = "medium"):
        """
        Log a security event.

        Args:
            event_type: Type of security event
            details: Details about the event
            severity: Severity level
        """
        self.logger.warning(
            f"Security event: {event_type}",
            event_type=event_type,
            details=details,
            severity=severity
        )

        # In a production system, you might also want to:
        # - Send alerts to security team
        # - Block the source IP
        # - Add to threat intelligence database
        # - Trigger incident response


def create_default_security_validator() -> SecurityValidator:
    """
    Create a default security validator instance.

    Returns:
        SecurityValidator instance
    """
    return SecurityValidator()


# Global instance
security_validator = create_default_security_validator()


def get_security_validator() -> SecurityValidator:
    """
    Get the global security validator instance.

    Returns:
        SecurityValidator instance
    """
    return security_validator


# Convenience functions
def validate_input(input_data: Union[str, Dict, List], context: str = "general") -> SecurityValidationResult:
    """
    Convenience function to validate input data for security threats.
    """
    return security_validator.validate_input(input_data, context)


def validate_url(url: str) -> SecurityValidationResult:
    """
    Convenience function to validate a URL for security threats.
    """
    return security_validator.validate_url(url)


def scan_content(content: str, content_type: str = "text") -> SecurityValidationResult:
    """
    Convenience function to scan content for security threats.
    """
    return security_validator.scan_content(content, content_type)


def validate_api_key(api_key: str) -> SecurityValidationResult:
    """
    Convenience function to validate an API key.
    """
    return security_validator.validate_api_key(api_key)


def sanitize_content(content: str, content_type: str = "text") -> str:
    """
    Convenience function to sanitize content.
    """
    return security_validator.sanitize_content(content, content_type)


# Export for use in other modules
__all__ = [
    'SecurityValidator',
    'SecurityScanType',
    'SecurityValidationResult',
    'create_default_security_validator',
    'get_security_validator',
    'validate_input',
    'validate_url',
    'scan_content',
    'validate_api_key',
    'sanitize_content',
    'security_validator'
]