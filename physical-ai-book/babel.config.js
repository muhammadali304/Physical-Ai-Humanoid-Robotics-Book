module.exports = {
  presets: [
    [
      '@docusaurus/core/lib/babel/preset',
      {
        // Enable modern JavaScript features
        runtime: 'automatic',
      },
    ],
  ],
  // Allow ES modules syntax
  sourceType: 'unambiguous', // This allows mixed module systems
  // Additional configuration for ES modules
  assumptions: {
    // This helps with ES module compatibility
    setPublicClassFields: true,
    privateFieldsAsProperties: true,
  },
};