# CI/CD Pipeline for GitHub Pages Deployment

This document describes the Continuous Integration and Continuous Deployment pipeline for the Physical AI & Humanoid Robotics Educational Book.

## Overview

The CI/CD pipeline automates:
- Code validation and testing
- Website building
- Deployment to GitHub Pages
- Security checks
- Link validation

## Pipeline Components

### 1. Continuous Integration (CI)

The CI pipeline runs on every push and pull request to ensure code quality:

#### Test Job
- Runs on multiple Node.js versions (18.x, 20.x)
- Builds the website to catch build errors
- Runs the link checker to validate external references
- Validates Docusaurus configuration

#### Security Job
- Performs npm security audit
- Checks for vulnerable dependencies

#### Lint Job
- Validates markdown links
- Ensures content quality

### 2. Continuous Deployment (CD)

The deployment pipeline runs automatically when changes are merged to the `main` branch:

- Builds the production website
- Deploys to GitHub Pages
- Maintains version history

## Workflow Files

### `.github/workflows/ci.yml`
- **Trigger**: Push and pull request events
- **Purpose**: Validate code changes
- **Jobs**: Test, security, and linting

### `.github/workflows/deploy.yml`
- **Trigger**: Push events to `main` branch
- **Purpose**: Deploy built site to GitHub Pages
- **Jobs**: Test and deploy

## Configuration

### Node.js Version
The pipeline uses Node.js 18.x LTS for deployment, ensuring compatibility with GitHub Pages build environment.

### Deployment Settings
- **Source**: `./build` directory after running `npm run build`
- **Target**: GitHub Pages via `peaceiris/actions-gh-pages`
- **Branch**: `gh-pages` (auto-generated)

## Environment Variables

The pipeline uses GitHub Actions secrets:
- `GITHUB_TOKEN`: Automatically provided by GitHub for deployment

## Custom Domain Support

To use a custom domain:
1. Uncomment the `cname` line in `.github/workflows/deploy.yml`
2. Replace `your-domain.com` with your actual domain
3. Add a `CNAME` file to your repository root

## Manual Deployment

If needed, you can manually trigger a deployment:

```bash
# Build the site locally
npm run build

# Deploy manually using Docusaurus command
GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
```

## Troubleshooting

### Build Failures
- Check for Node.js version compatibility
- Verify all dependencies are properly specified
- Review Docusaurus configuration for errors

### Deployment Issues
- Ensure the `organizationName` and `projectName` in `docusaurus.config.js` are correct
- Verify GitHub Pages is enabled in repository settings
- Check that the `GITHUB_TOKEN` has proper permissions

## Best Practices

1. **Always test locally** before pushing changes
2. **Use pull requests** for code review
3. **Monitor workflow runs** for any issues
4. **Keep dependencies updated** to avoid security vulnerabilities
5. **Validate external links** regularly using the link checker