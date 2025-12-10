#!/usr/bin/env node

const { build } = require('@docusaurus/core');
const path = require('path');

const siteDir = path.join(__dirname, '..');
const config = path.join(siteDir, 'docusaurus.config.js');

async function runBuild() {
  try {
    await build(siteDir, {
      bundleAnalyzer: false,
      outDir: path.join(siteDir, 'build'),
      config,
      minify: true,
    });
    console.log('Build completed successfully!');
  } catch (error) {
    console.error('Build failed:', error);
    process.exit(1);
  }
}

runBuild();