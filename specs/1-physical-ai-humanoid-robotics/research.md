# Research: Physical AI & Humanoid Robotics Educational Book

## Decision: ROS 2 Distribution
- **Options:** Humble Hawksbill (LTS), Iron Irwini (Latest), Rolling (Bleeding Edge)
- **Choice:** Humble Hawksbill
- **Rationale:** LTS support until 2027, stable ecosystem, best hardware compatibility, most documentation and community support
- **Tradeoffs:** Miss some newer features in Iron, but gain stability and long-term support
- **Documented in:** Chapter 2 (Setup)

## Decision: Primary Simulation Platform
- **Options:** Gazebo Classic, Gazebo Sim (Ignition), Isaac Sim, Unity
- **Choice:** Gazebo for basics, Isaac Sim for advanced
- **Rationale:** Gazebo has best ROS 2 integration for learning, Isaac for AI workflows and advanced perception
- **Tradeoffs:** Learning curve for two platforms, but covers full spectrum from basic to advanced
- **Documented in:** Chapter 6 (Simulation)

## Decision: Hardware Budget Tier
- **Options:** Simulation-only, Budget ($700 kit), Standard ($3K), Premium ($16K+)
- **Choice:** Present all tiers, recommend Budget as minimum physical
- **Rationale:** Accessibility while maintaining "Physical AI" promise, realistic for educational institutions
- **Tradeoffs:** Cannot do full humanoid deployment on budget tier, but provides hands-on experience
- **Documented in:** Appendix A (Hardware Guide)

## Decision: LLM Integration Approach
- **Options:** Local models (Llama), API-based (GPT-4, Claude), Custom fine-tuned
- **Choice:** API-based (OpenAI/Anthropic) with local alternatives mentioned
- **Rationale:** Easier for students, no training infrastructure needed, consistent performance
- **Tradeoffs:** Cost per API call, internet dependency, but reduces setup complexity
- **Documented in:** Chapter 12 (VLA)

## Decision: Deployment Target
- **Options:** GitHub Pages (static), Vercel, Netlify, Self-hosted
- **Choice:** GitHub Pages
- **Rationale:** Free, integrated with repo, simple CI/CD, sufficient for Docusaurus static site
- **Tradeoffs:** Limited to static sites, but fine for documentation-based book
- **Documented in:** README and CI/CD setup

## Decision: Target Audience
- **Options:** Undergraduate students, Graduate students, Professionals, Hobbyists
- **Choice:** Advanced undergraduate/graduate students with AI/ML background
- **Rationale:** Matches the complexity of the content, assumes Python and ML fundamentals
- **Tradeoffs:** May be too advanced for beginners, but appropriate for target educational level
- **Documented in:** Introduction

## Decision: Educational Structure
- **Options:** Linear chapters, Modular topics, Project-based learning, Spiral curriculum
- **Choice:** Progressive complexity from foundational to advanced (13-week curriculum)
- **Rationale:** Follows academic calendar, builds concepts systematically, includes hands-on exercises
- **Tradeoffs:** Requires commitment to full sequence, but ensures comprehensive understanding
- **Documented in:** Chapter structure

## Technology Research Findings

### ROS 2 Ecosystem
- ROS 2 Humble Hawksbill is the LTS version with 5-year support (2027)
- Extensive documentation available at docs.ros.org
- Strong integration with Gazebo for simulation
- Isaac ROS provides perception and navigation packages for NVIDIA hardware

### Simulation Platforms
- Gazebo has mature ROS 2 integration with well-documented tutorials
- Isaac Sim offers advanced AI training capabilities and sim-to-real transfer
- Unity provides alternative with strong graphics but less ROS integration
- Gazebo Classic is being phased out in favor of Gazebo Sim (Ignition)

### Hardware Options
- Budget tier: TurtleBot 4 or similar differential drive robots (~$700)
- Standard tier: Quadruped robots like Unitree Go1 or similar (~$3K)
- Premium tier: Full humanoid robots like Unitree H1 or Boston Dynamics clones (~$16K+)
- Simulation can provide most learning outcomes without hardware

### Docusaurus for Educational Content
- Supports MDX for interactive components
- Built-in search functionality
- Mobile-responsive design
- Versioning support for content updates
- Good for technical documentation with code examples

### Performance Requirements
- Physics simulation: Target 30 FPS for realistic interaction
- Perception pipeline: <500ms for real-time processing
- Setup time: <2 hours for complete environment
- Code execution: 95% success rate on clean Ubuntu 22.04