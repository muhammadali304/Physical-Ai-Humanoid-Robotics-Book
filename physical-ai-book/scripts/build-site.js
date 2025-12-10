#!/usr/bin/env node

const path = require('path');

// Dynamically import @docusaurus/core to avoid issues with module resolution
async function runBuild() {
  try {
    const docusaurus = await import('@docusaurus/core');
    const { build } = docusaurus;

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