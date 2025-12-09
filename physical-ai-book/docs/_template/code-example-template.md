# Code Example Template

This template provides a standardized format for all code examples in the Physical AI & Humanoid Robotics educational book.

## File Structure Convention

```
examples/
├── category/
│   ├── subcategory/
│   │   ├── example_name.language_extension
│   │   ├── example_name_README.md
│   │   └── requirements.txt (if needed)
```

## Code Example Template

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example: [Brief description of what this example demonstrates]

Description:
    [Detailed description of the example's purpose, functionality,
    and educational value]

Author: Physical AI & Humanoid Robotics Educational Book
Date: YYYY-MM-DD
Version: 1.0
License: MIT

Learning Objectives:
    1. [Objective 1]
    2. [Objective 2]
    3. [Objective 3]

Prerequisites:
    - [Required knowledge/skills]
    - [Required software/packages installed]
    - [Required setup steps completed]

Usage:
    python3 example_name.py [optional_arguments]

Arguments:
    --help          Show help message and exit
    --param1        Description of parameter 1
    --param2        Description of parameter 2
"""

# Standard library imports
import sys
import os
import argparse
import logging
from typing import Any, Dict, List, Optional, Tuple

# Third-party imports
import numpy as np
import rospy  # For ROS examples
# Add other imports as needed

# Local imports
# Import local modules if any

# Constants
DEFAULT_PARAM1 = "default_value"
DEFAULT_PARAM2 = 42
EXAMPLE_CONSTANT = "example_value"

# Global variables
logger = None

def setup_logging(verbosity: int = 0) -> logging.Logger:
    """
    Set up logging configuration.

    Args:
        verbosity: Level of verbosity (0=INFO, 1=DEBUG, 2+=TRACE)

    Returns:
        Configured logger instance
    """
    log_level = logging.INFO
    if verbosity >= 1:
        log_level = logging.DEBUG

    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )

    return logging.getLogger(__name__)

def validate_inputs(args: argparse.Namespace) -> bool:
    """
    Validate input arguments.

    Args:
        args: Parsed command-line arguments

    Returns:
        True if inputs are valid, False otherwise
    """
    # Add input validation logic here
    if not hasattr(args, 'param1'):
        print("Error: param1 is required")
        return False

    return True

def example_function(param1: str, param2: int = DEFAULT_PARAM2) -> Any:
    """
    Example function demonstrating the main functionality.

    This function demonstrates [specific concept/principle].

    Args:
        param1: Description of parameter 1
        param2: Description of parameter 2 (default: DEFAULT_PARAM2)

    Returns:
        Description of return value

    Raises:
        ValueError: If invalid parameters are provided
        RuntimeError: If the operation fails
    """
    # Function implementation
    result = f"Processing {param1} with {param2}"

    # Add detailed implementation here
    # Include error handling and edge cases

    return result

def main():
    """
    Main function to execute the example.

    Parses command-line arguments, validates inputs,
    executes the main functionality, and handles cleanup.
    """
    global logger

    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Example: [Brief description]",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --param1 value1
  %(prog)s --param1 value1 --param2 100
        """
    )

    parser.add_argument(
        "--param1",
        type=str,
        required=True,
        help="Description of parameter 1"
    )

    parser.add_argument(
        "--param2",
        type=int,
        default=DEFAULT_PARAM2,
        help=f"Description of parameter 2 (default: {DEFAULT_PARAM2})"
    )

    parser.add_argument(
        "-v", "--verbose",
        action="count",
        default=0,
        help="Increase verbosity (use -v, -vv, or -vvv for more detail)"
    )

    args = parser.parse_args()

    # Set up logging
    logger = setup_logging(args.verbose)
    logger.info("Starting example execution")

    # Validate inputs
    if not validate_inputs(args):
        logger.error("Input validation failed")
        sys.exit(1)

    try:
        # Execute main functionality
        result = example_function(args.param1, args.param2)
        print(f"Result: {result}")

        logger.info("Example executed successfully")

    except ValueError as e:
        logger.error(f"Value error: {e}")
        sys.exit(1)
    except RuntimeError as e:
        logger.error(f"Runtime error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        logger.info("Execution interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        sys.exit(1)

    return 0

if __name__ == "__main__":
    sys.exit(main())
```

## README Template for Code Examples

```markdown
# [Example Name]

## Overview
Brief description of what this example demonstrates and its educational purpose.

## Prerequisites
- [List of requirements]
- [Software dependencies]
- [System requirements]

## Setup
Instructions for setting up the example:
```bash
# Installation commands if needed
pip install -r requirements.txt
```

## Usage
How to run the example:
```bash
python3 example_name.py --param1 value1 --param2 value2
```

## Expected Output
Description of what users should expect to see when running the example.

## Key Concepts Demonstrated
- [Concept 1]
- [Concept 2]
- [Concept 3]

## Troubleshooting
Common issues and solutions:
- **Issue**: [Problem description]
  **Solution**: [How to fix it]

## Next Steps
How this example connects to other concepts in the book.

## References
- [Relevant documentation]
- [Additional resources]
- [Related examples]
```

## Requirements File Template

```txt
# Requirements for [Example Name]
# Generated for Physical AI & Humanoid Robotics Educational Book
# Date: YYYY-MM-DD

# Core dependencies
numpy>=1.19.0
scipy>=1.5.0

# ROS/Robotics specific (if applicable)
rospy>=1.15.0
geometry_msgs
sensor_msgs

# Visualization (if needed)
matplotlib>=3.3.0
opencv-python>=4.5.0

# Testing (if needed)
pytest>=6.0.0
pytest-cov>=2.10.0
```

## Testing Template

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for [Example Name]

This module contains unit tests for the example code to ensure
correct functionality and educational value.
"""

import unittest
import sys
import os
from unittest.mock import patch, MagicMock

# Add the examples directory to the path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import the module to test
from example_name import example_function, validate_inputs

class TestExampleName(unittest.TestCase):
    """Test cases for example_name module."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def tearDown(self):
        """Clean up after each test method."""
        pass

    def test_example_function_valid_input(self):
        """Test example_function with valid input."""
        result = example_function("test", 42)
        self.assertIsNotNone(result)
        self.assertIn("Processing test with 42", result)

    def test_example_function_default_param(self):
        """Test example_function with default parameter."""
        result = example_function("test")
        self.assertIsNotNone(result)
        self.assertIn("Processing test with 42", result)

    def test_validate_inputs_valid(self):
        """Test validate_inputs with valid arguments."""
        # Create mock args object
        class MockArgs:
            param1 = "test_value"

        args = MockArgs()
        result = validate_inputs(args)
        self.assertTrue(result)

    def test_validate_inputs_missing_param(self):
        """Test validate_inputs with missing required parameter."""
        # Create mock args object without required param
        class MockArgs:
            pass

        args = MockArgs()
        result = validate_inputs(args)
        self.assertFalse(result)

if __name__ == '__main__':
    unittest.main()
```

## Documentation Standards

### Inline Comments
- Use clear, concise comments that explain *why* not *what*
- Document complex algorithms and non-obvious logic
- Use docstrings for all functions, classes, and modules

### Error Handling
- Include appropriate error handling for all functions
- Provide meaningful error messages
- Follow consistent error handling patterns

### Code Style
- Follow PEP 8 guidelines for Python
- Use consistent naming conventions
- Keep functions focused and modular
- Include type hints where appropriate

### Educational Value
- Include explanations of key concepts within comments
- Provide references to relevant book sections
- Add links to external documentation when appropriate