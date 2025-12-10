# Research: AI-Native Textbook — Physical AI & Humanoid Robotics

## Decision: Docusaurus as Documentation Platform
**Rationale**: Docusaurus is the optimal choice for creating the AI-Native Textbook based on the feature specification requirements. It provides:
- Static site generation with Markdown support
- Built-in GitHub Pages deployment capabilities
- Navigation sidebar support for structured content
- Versioning capabilities if needed in the future
- Strong documentation ecosystem

**Alternatives considered**:
- GitBook: Good but less flexible than Docusaurus
- Custom React app: More complex, requires more maintenance
- Hugo/Jekyll: Good alternatives but Docusaurus has better Markdown integration for technical content

## Decision: Technology Stack for Textbook Development
**Language/Version**: Markdown for content, JavaScript/Node.js for Docusaurus configuration
- Node.js 18+ (LTS recommended for Docusaurus)
- Docusaurus 3.x (latest stable version)

**Primary Dependencies**:
- Docusaurus core packages
- Docusaurus GitHub Pages plugin
- React for any custom components
- GitHub Actions for CI/CD

**Testing**:
- Docusaurus built-in build validation (npm run build)
- Markdown linting tools
- Link checking tools

**Target Platform**:
- Web-based documentation site
- GitHub Pages hosting
- Responsive design for various devices

**Performance Goals**:
- Fast loading times (<3 seconds as per spec)
- Optimized for documentation browsing
- Mobile-friendly navigation

## Decision: Project Structure
**Rationale**: The Docusaurus standard project structure will be used with the following organization:
- docs/: Contains all textbook content organized by modules and weeks
- src/: Custom components if needed
- static/: Static assets (images, diagrams, etc.)
- docusaurus.config.js: Main configuration file
- sidebars.js: Navigation structure
- package.json: Dependencies and scripts

## Decision: GitHub Pages Deployment
**Rationale**: GitHub Pages deployment will be configured using GitHub Actions workflow:
- Automatic deployment on push to main branch
- Proper organizationName and projectName settings
- Correct baseUrl configuration
- Build validation before deployment

## Decision: Content Organization
**Rationale**: Content will be organized to match the 4 Modules and 13 Weeks curriculum:
- Module 1: The Robotic Nervous System (ROS 2) - 3 weeks
- Module 2: The Digital Twin (Gazebo & Unity) - 2 weeks
- Module 3: The AI-Robot Brain (NVIDIA Isaac™) - 3 weeks
- Module 4: Vision-Language-Action (VLA) - 4 weeks (including capstone)

Each week will be a separate Markdown file with proper navigation structure in sidebars.js.

## Decision: Technical Accuracy Requirements
**Rationale**: All content must be technically accurate for the robotics frameworks mentioned:
- ROS 2 commands, nodes, topics, services, packages
- Gazebo physics simulation, gravity, collisions
- NVIDIA Isaac Sim, VSLAM, Nav2, reinforcement learning
- VLA integration with OpenAI Whisper and LLMs

Content will be validated against official documentation for each framework.