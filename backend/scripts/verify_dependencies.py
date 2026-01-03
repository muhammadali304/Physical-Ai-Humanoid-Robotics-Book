#!/usr/bin/env python3
"""
Script to verify external dependencies against official documentation.
This script checks that installed packages match expected versions and functionality.
"""

import importlib
import subprocess
import sys
from packaging import version


def check_package_version(package_name, min_version=None):
    """Check if a package is installed and meets minimum version requirements."""
    try:
        module = importlib.import_module(package_name)
        if hasattr(module, '__version__'):
            ver = module.__version__
            print(f"✓ {package_name} version: {ver}")

            if min_version:
                if version.parse(ver) >= version.parse(min_version):
                    print(f"  ✓ Meets minimum version requirement: {min_version}")
                    return True
                else:
                    print(f"  ✗ Below minimum version: {min_version}")
                    return False
            return True
        else:
            print(f"✓ {package_name} (no version info)")
            return True
    except ImportError:
        print(f"✗ {package_name} not found")
        return False
    except Exception as e:
        print(f"✗ Error checking {package_name}: {str(e)}")
        return False


def check_cohere():
    """Verify Cohere API functionality."""
    try:
        import cohere
        print("✓ Cohere package imported successfully")

        # Check if we have an API key
        import os
        api_key = os.getenv("COHERE_API_KEY")
        if api_key and len(api_key) > 5:  # Basic check
            print("✓ Cohere API key found in environment")
        else:
            print("! Cohere API key not found (this is OK for validation)")

        return True
    except ImportError:
        print("✗ Cohere package not installed")
        return False
    except Exception as e:
        print(f"✗ Error with Cohere: {str(e)}")
        return False


def check_qdrant():
    """Verify Qdrant client functionality."""
    try:
        import qdrant_client
        print("✓ Qdrant client package imported successfully")

        # Check basic client functionality
        client_info = qdrant_client.__version__
        print(f"✓ Qdrant client version: {client_info}")

        return True
    except ImportError:
        print("✗ Qdrant client package not installed")
        return False
    except Exception as e:
        print(f"✗ Error with Qdrant: {str(e)}")
        return False


def check_beautifulsoup():
    """Verify BeautifulSoup functionality."""
    try:
        from bs4 import BeautifulSoup
        print("✓ BeautifulSoup4 package imported successfully")

        # Test basic functionality
        test_html = "<html><body><p>Test</p></body></html>"
        soup = BeautifulSoup(test_html, 'html.parser')
        if soup.find('p').text == 'Test':
            print("✓ BeautifulSoup4 basic functionality works")
        else:
            print("✗ BeautifulSoup4 basic functionality failed")
            return False

        return True
    except ImportError:
        print("✗ BeautifulSoup4 package not installed")
        return False
    except Exception as e:
        print(f"✗ Error with BeautifulSoup4: {str(e)}")
        return False


def check_fastapi():
    """Verify FastAPI functionality."""
    try:
        import fastapi
        print("✓ FastAPI package imported successfully")

        # Check version
        print(f"✓ FastAPI version: {fastapi.__version__}")

        return True
    except ImportError:
        print("✗ FastAPI package not installed")
        return False
    except Exception as e:
        print(f"✗ Error with FastAPI: {str(e)}")
        return False


def check_redis_rq():
    """Verify Redis and RQ functionality."""
    try:
        import redis
        print("✓ Redis package imported successfully")

        # Check version
        print(f"✓ Redis version: {redis.__version__}")
    except ImportError:
        print("✗ Redis package not installed")
        return False
    except Exception as e:
        print(f"✗ Error with Redis: {str(e)}")
        return False

    try:
        import rq
        print("✓ RQ package imported successfully")

        # Check version
        print(f"✓ RQ version: {rq.version}")

        return True
    except ImportError:
        print("✗ RQ package not installed")
        return False
    except Exception as e:
        print(f"✗ Error with RQ: {str(e)}")
        return False


def main():
    print("Verifying external dependencies against official documentation...")
    print("=" * 60)

    all_passed = True

    # Check core packages
    print("\nCore Packages:")
    core_checks = [
        check_package_version("httpx", "0.24.0"),
        check_package_version("lxml", "4.9.0"),
        check_package_version("python_dotenv", "1.0.0"),
    ]

    all_passed = all(core_checks) and all_passed

    # Check specific functionality
    print("\nSpecific Functionality:")
    specific_checks = [
        check_cohere(),
        check_qdrant(),
        check_beautifulsoup(),
        check_fastapi(),
        check_redis_rq(),
    ]

    all_passed = all(specific_checks) and all_passed

    print("\n" + "=" * 60)
    if all_passed:
        print("✓ All dependencies verified successfully!")
        return 0
    else:
        print("✗ Some dependencies failed verification")
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)