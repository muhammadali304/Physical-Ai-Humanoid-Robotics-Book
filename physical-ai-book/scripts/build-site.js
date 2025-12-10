#!/usr/bin/env node

const path = require('path');
const fs = require('fs');

// Try multiple approaches to load @docusaurus/core
async function runBuild() {
  try {
    let build;

    // Try to load @docusaurus/core using require.resolve first
    try {
      const docusaurusCorePath = require.resolve('@docusaurus/core');
      const docusaurus = require(docusaurusCorePath);
      build = docusaurus.build;
    } catch (e) {
      // If require.resolve fails, try dynamic import
      try {
        const docusaurus = await import('@docusaurus/core');
        build = docusaurus.build;
      } catch (e2) {
        // If both fail, try to find it in node_modules
        const docusaurusCorePath = path.join(__dirname, '../node_modules/@docusaurus/core/lib/index.js');
        if (fs.existsSync(docusaurusCorePath)) {
          const docusaurus = require(docusaurusCorePath);
          build = docusaurus.build;
        } else {
          throw new Error(`@docusaurus/core not found at expected locations: ${e.message}, ${e2.message}`);
        }
      }
    }

    const siteDir = path.join(__dirname, '..');
    const configPath = path.join(siteDir, 'docusaurus.config.js');

    await build(siteDir, {
      bundleAnalyzer: false,
      outDir: path.join(siteDir, 'build'),
      config: configPath,
      minify: true,
    });
    console.log('Build completed successfully!');
  } catch (error) {
    console.error('Build failed:', error);
    process.exit(1);
  }
}

runBuild();