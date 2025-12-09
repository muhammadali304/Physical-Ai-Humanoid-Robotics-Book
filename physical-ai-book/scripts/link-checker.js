#!/usr/bin/env node

/**
 * External Link Checker for Physical AI & Humanoid Robotics Educational Book
 *
 * This script scans all markdown files in the docs directory for external links
 * and verifies that they return a 200 status code (or acceptable redirect codes).
 *
 * Usage:
 *   node link-checker.js
 *
 * Options:
 *   --verbose, -v: Show detailed output for each link checked
 *   --fix, -f: Attempt to fix common link issues (coming soon)
 *   --timeout, -t: Set timeout for HTTP requests in milliseconds (default: 10000)
 */

const fs = require('fs');
const path = require('path');
const https = require('https');
const http = require('http');
const { URL } = require('url');

// Configuration
const DOCS_DIR = './docs';
const IGNORE_PATTERNS = [
  // Internal links (relative or absolute)
  /^(\.\.\/|\.\/|\/)/,
  // Anchor links
  /^#/,
  // Mailto links
  /^mailto:/,
  // File references
  /\.(pdf|jpg|jpeg|png|gif|svg|zip|tar|gz)$/i,
  // Localhost/development URLs
  /localhost|127\.0\.0\.1|0\.0\.0\.0/
];
const VALID_STATUS_CODES = [200, 201, 202, 203, 204, 301, 302, 303, 307, 308];
const DEFAULT_TIMEOUT = 10000; // 10 seconds

// Command line arguments
const args = process.argv.slice(2);
const verbose = args.includes('--verbose') || args.includes('-v');
const fixMode = args.includes('--fix') || args.includes('-f');
const timeoutArg = args.find(arg => arg.startsWith('--timeout=') || arg.startsWith('-t='));
const timeout = timeoutArg ? parseInt(timeoutArg.split('=')[1]) : DEFAULT_TIMEOUT;

let totalLinks = 0;
let brokenLinks = [];
let checkedLinks = new Set();

/**
 * Extracts all external links from a markdown file
 * @param {string} content - The markdown file content
 * @returns {Array<string>} Array of external URLs found
 */
function extractLinks(content) {
  const links = [];

  // Match [text](url) pattern
  const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
  let match;
  while ((match = linkRegex.exec(content)) !== null) {
    const url = match[2].trim();
    if (isExternalLink(url)) {
      links.push(url);
    }
  }

  // Match <http://example.com> pattern
  const autoLinkRegex = /<((?:http|https):\/\/[^>]+)>/g;
  while ((match = autoLinkRegex.exec(content)) !== null) {
    const url = match[1].trim();
    if (isExternalLink(url)) {
      links.push(url);
    }
  }

  return links;
}

/**
 * Checks if a URL is an external link
 * @param {string} url - The URL to check
 * @returns {boolean} True if it's an external link
 */
function isExternalLink(url) {
  // Check if it matches any ignore pattern
  for (const pattern of IGNORE_PATTERNS) {
    if (pattern.test(url)) {
      return false;
    }
  }

  // Check if it's a valid HTTP/HTTPS URL
  try {
    const parsed = new URL(url);
    return parsed.protocol === 'http:' || parsed.protocol === 'https:';
  } catch (e) {
    // If it's not a valid URL, it's not an external link
    return false;
  }
}

/**
 * Checks if a URL is accessible by making an HTTP request
 * @param {string} url - The URL to check
 * @returns {Promise<Object>} Object with status and message
 */
function checkUrl(url) {
  return new Promise((resolve) => {
    const parsedUrl = new URL(url);
    const options = {
      hostname: parsedUrl.hostname,
      port: parsedUrl.port,
      path: parsedUrl.pathname + parsedUrl.search + parsedUrl.hash,
      method: 'HEAD', // Use HEAD to minimize data transfer
      timeout: timeout,
      headers: {
        'User-Agent': 'Physical-AI-Book-Link-Checker/1.0'
      }
    };

    const client = parsedUrl.protocol === 'https:' ? https : http;

    const request = client.request(options, (response) => {
      resolve({
        url,
        status: response.statusCode,
        statusText: response.statusMessage,
        valid: VALID_STATUS_CODES.includes(response.statusCode),
        redirectUrl: response.headers.location
      });
    });

    request.on('error', (error) => {
      resolve({
        url,
        status: null,
        statusText: error.message,
        valid: false,
        error: error
      });
    });

    request.on('timeout', () => {
      request.destroy();
      resolve({
        url,
        status: null,
        statusText: 'Request timeout',
        valid: false,
        error: new Error('Request timeout')
      });
    });

    request.end();
  });
}

/**
 * Recursively gets all markdown files in a directory
 * @param {string} dir - The directory to scan
 * @returns {Array<string>} Array of markdown file paths
 */
function getMarkdownFiles(dir) {
  const files = [];
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const fullPath = path.join(dir, item);
    const stat = fs.statSync(fullPath);

    if (stat.isDirectory()) {
      files.push(...getMarkdownFiles(fullPath));
    } else if (path.extname(item) === '.md') {
      files.push(fullPath);
    }
  }

  return files;
}

/**
 * Main function to run the link checker
 */
async function runLinkChecker() {
  console.log('🔍 Starting external link check for Physical AI & Humanoid Robotics Educational Book...\n');

  const markdownFiles = getMarkdownFiles(DOCS_DIR);
  console.log(`📋 Found ${markdownFiles.length} markdown files to scan\n`);

  const linkPromises = [];
  const fileLinkMap = new Map();

  // First pass: collect all links from all files
  for (const file of markdownFiles) {
    try {
      const content = fs.readFileSync(file, 'utf8');
      const links = extractLinks(content);

      if (links.length > 0) {
        fileLinkMap.set(file, links);
        totalLinks += links.length;

        for (const link of links) {
          // Only check each unique URL once
          if (!checkedLinks.has(link)) {
            checkedLinks.add(link);
            linkPromises.push(checkUrl(link));
          }
        }
      }
    } catch (error) {
      console.error(`❌ Error reading file ${file}:`, error.message);
    }
  }

  console.log(`🔗 Found ${totalLinks} total external links (${checkedLinks.size} unique URLs)`);
  console.log(`⏳ Checking ${checkedLinks.size} unique URLs...\n`);

  // Second pass: check all collected links
  const results = await Promise.allSettled(linkPromises);
  const linkStatusMap = new Map();

  for (let i = 0; i < results.length; i++) {
    const result = results[i];
    if (result.status === 'fulfilled') {
      const data = result.value;
      linkStatusMap.set(data.url, data);

      if (!data.valid) {
        brokenLinks.push(data);
      }

      if (verbose) {
        const statusSymbol = data.valid ? '✅' : '❌';
        console.log(`${statusSymbol} ${data.url} - ${data.status || 'N/A'} ${data.statusText}`);
      }
    } else {
      console.error(`❌ Error checking link:`, result.reason);
    }
  }

  // Third pass: map results back to files
  const brokenFileMap = new Map();
  for (const [file, links] of fileLinkMap) {
    const brokenFileLinks = links.filter(link => {
      const status = linkStatusMap.get(link);
      return status && !status.valid;
    });

    if (brokenFileLinks.length > 0) {
      brokenFileMap.set(file, brokenFileLinks);
    }
  }

  // Output results
  console.log('\n📊 Results:');
  console.log(`✅ Valid links: ${checkedLinks.size - brokenLinks.length}`);
  console.log(`❌ Broken links: ${brokenLinks.length}`);
  console.log(`📄 Files with broken links: ${brokenFileMap.size}\n`);

  if (brokenLinks.length > 0) {
    console.log('❌ Broken Links Details:');
    for (const link of brokenLinks) {
      console.log(`  • ${link.url}`);
      console.log(`    Status: ${link.status || 'N/A'} ${link.statusText}`);
      console.log(`    File(s): ${Array.from(fileLinkMap.entries())
        .filter(([file, links]) => links.includes(link.url))
        .map(([file]) => path.relative('.', file))
        .join(', ')}`);
      console.log('');
    }

    console.log('⚠️  Warning: Some external links are broken. Please update these links in the documentation.');
    process.exit(1); // Exit with error code if there are broken links
  } else {
    console.log('✅ All external links are valid!');
  }
}

// Run the link checker
runLinkChecker().catch(error => {
  console.error('❌ Error running link checker:', error);
  process.exit(1);
});