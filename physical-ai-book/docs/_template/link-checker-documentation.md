# Automated Link Checker

The Physical AI & Humanoid Robotics Educational Book includes an automated link checker to ensure all external references remain valid.

## Purpose

This tool scans all markdown documentation files for external links and verifies their accessibility. It helps maintain the quality and reliability of external references throughout the book.

## How It Works

1. **Scans** all `.md` files in the `docs/` directory
2. **Extracts** external HTTP/HTTPS links (ignoring internal links, images, and local references)
3. **Checks** each unique URL with an HTTP HEAD request
4. **Reports** broken links with details about the error
5. **Lists** which files contain broken links

## Usage

Run the link checker from the project root:

```bash
node scripts/link-checker.js
```

### Options

- `--verbose` or `-v`: Show detailed output for each link checked
- `--timeout=N` or `-t=N`: Set timeout for HTTP requests in milliseconds (default: 10000)

### Examples

```bash
# Basic check
node scripts/link-checker.js

# Verbose output with detailed results
node scripts/link-checker.js --verbose

# Set timeout to 15 seconds
node scripts/link-checker.js --timeout=15000

# Combine options
node scripts/link-checker.js -v --timeout=15000
```

## Configuration

The link checker is configured with:

- **Valid status codes**: 200, 201, 202, 203, 204, 301, 302, 303, 307, 308
- **Default timeout**: 10 seconds per request
- **Ignored patterns**: Internal links, mailto:, local files, localhost addresses

## Integration

Add to your CI/CD pipeline to automatically check for broken links:

```bash
# In your CI script
if ! node scripts/link-checker.js; then
  echo "Broken links found! Please fix before merging."
  exit 1
fi
```

## Output

The tool provides:
- Count of total and unique external links
- Count of valid and broken links
- Detailed list of broken links with status codes
- Files containing broken links
- Exit code 1 if broken links are found (useful for CI/CD)

## Maintenance

Run the link checker regularly to maintain link quality:
- Before each major release
- As part of CI/CD pipeline
- Periodically (e.g., monthly) to catch link rot