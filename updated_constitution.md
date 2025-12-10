<!--
SYNC IMPACT REPORT
Version change: 0.0.0 → 1.0.0
List of modified principles: None (all new principles based on user input)
Added sections: Technical Standards, Source Requirements, Code Standards, Content Structure, Hardware Documentation, Deployment Requirements, Prohibited Practices, Success Criteria (Technical Validation, Educational Effectiveness, Production Quality, Documentation Standards), Quality Assurance Checklist (Pre-Publication, Post-Publication Maintenance)
Removed sections: None (all placeholders replaced with concrete content)
Templates requiring updates: ✅ .specify/templates/plan-template.md (Constitution Check section needs alignment) / ⚠ pending .specify/templates/spec-template.md / ⚠ pending .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Educational book on Physical AI & Humanoid Robotics Constitution

## Core Principles

### Technical accuracy
Technical accuracy through verification against official documentation and industry standards. All technical specifications must reference official documentation (ROS 2, NVIDIA Isaac, Gazebo) and code examples must be tested and functional.

### Educational clarity
Educational clarity for advanced undergraduate/graduate students with AI/ML background. Content must be accessible to students with foundation in Python, machine learning fundamentals, and basic robotics concepts, with clear learning objectives for each module.

### Practical applicability
Practical applicability with hands-on examples, code snippets, and deployment instructions. Include both simulation-first and hardware deployment paths with troubleshooting sections for common setup issues.

### Progressive complexity
Progressive complexity from foundational concepts to advanced implementations. Content must follow a progressive difficulty curve with no sudden complexity jumps, starting with basic concepts and building to advanced implementations.

### Cost transparency
Cost transparency with realistic hardware requirements and alternatives. Provide three-tier hardware approach (Budget/Standard/Premium) with exact model numbers and current pricing (with date stamps), and document cloud alternatives for resource-constrained scenarios.

### Content Quality
All technical specifications must reference official documentation (ROS 2, NVIDIA Isaac, Gazebo). Code examples must be tested and functional. Hardware recommendations based on verified minimum requirements. Include both simulation-first and hardware deployment paths. Provide troubleshooting sections for common setup issues.

## Technical Standards

### ROS 2 version
ROS 2 version: Humble/Iron (LTS releases) as the primary target. Operating system: Ubuntu 22.04 LTS as primary target. Code style: Follow ROS 2 Python style guide (PEP 8 compliant). Documentation format: Markdown with MDX for interactive components. Citation format: IEEE style for technical references, inline links for documentation.

### Source Requirements
Official documentation (ROS 2 docs, NVIDIA developer docs) as primary sources. Industry whitepapers and technical blogs from recognized authorities. Academic papers for theoretical foundations (reinforcement learning, SLAM, kinematics). GitHub repositories with active maintenance and proper licensing. Minimum 70% sources from last 3 years (rapidly evolving field).

### Code Standards
All code blocks must specify language for syntax highlighting. Include installation commands and dependency lists. Provide both minimal and production-ready examples. Add inline comments explaining non-obvious implementations. Test all commands on clean Ubuntu 22.04 installation.

## Content Structure

### Format
Format: Docusaurus documentation site with sidebar navigation. Target length: 12-15 comprehensive chapters (covering 13-week curriculum). Code-to-explanation ratio: ~40% code/commands, 60% explanation. Visual aids: Minimum 2-3 diagrams per module (architecture, data flow, system design).

### Hardware Documentation
Provide three-tier approach: Budget/Standard/Premium. Include exact model numbers and current pricing (with date stamps). Document cloud alternatives for resource-constrained scenarios. Warn about incompatible hardware explicitly.

### Deployment Requirements
Platform: GitHub Pages with custom domain support. Build tool: Docusaurus v3.x. CI/CD: GitHub Actions for automated deployment. Accessibility: WCAG 2.1 AA compliance for documentation. Mobile responsive: All content must render on tablets/phones.

## Prohibited Practices

No untested code examples. No outdated package versions without migration notes. No hardware recommendations without availability verification. No "works on my machine" explanations. No copy-pasting large code blocks without explanation. No proprietary content without proper licensing.

## Success Criteria

### Technical Validation
All ROS 2 commands tested on Ubuntu 22.04. Isaac Sim examples verified on RTX 40-series GPUs. Hardware specifications validated against official vendor docs. Cloud deployment instructions tested on AWS/Azure. All external links verified and working (no 404s).

### Educational Effectiveness
Clear learning objectives for each module. Progressive difficulty curve (no sudden complexity jumps). Hands-on exercises with solution keys. Troubleshooting sections for top 5 common errors per module. Capstone project with grading rubric.

### Production Quality
Zero broken links or missing images. Consistent formatting throughout all chapters. Working search functionality. Load time under 3 seconds on 4G connection. Successfully deploys to GitHub Pages without errors.

### Documentation Standards
README with quick start guide. Contributing guidelines for community improvements. License file (CC BY-SA 4.0 for content, MIT for code). Changelog tracking major updates. Issue templates for bug reports and content suggestions.

## Quality Assurance Checklist

### Pre-Publication
Run Docusaurus build with no warnings. Test all installation commands on fresh VM. Verify all hardware prices within last 30 days. Check all external links (automated link checker). Peer review by at least one robotics practitioner. Accessibility audit using axe DevTools. Mobile rendering test on iOS and Android.

### Post-Publication Maintenance
Monthly link checker run. Quarterly hardware price updates. Bi-annual review of ROS 2/Isaac versions. Community issue response within 48 hours. Version tags for major curriculum updates.

## Governance

All content must follow these principles and standards. Amendments require documentation of changes, approval from project maintainers, and migration plan for existing content. All PRs/reviews must verify compliance with technical accuracy, educational clarity, and practical applicability. Content complexity must be justified with clear educational value. Use README.md for runtime development guidance.

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08