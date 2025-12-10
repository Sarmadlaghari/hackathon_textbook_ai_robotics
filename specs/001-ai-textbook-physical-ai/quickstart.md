# Quickstart Guide: AI-Native Textbook — Physical AI & Humanoid Robotics

## Prerequisites

- Node.js 18+ (LTS version recommended)
- npm or yarn package manager
- Git for version control
- GitHub account for deployment

## Setup Instructions

### 1. Clone and Initialize the Repository

```bash
git clone <repository-url>
cd my-website
npm install
```

### 2. Start Development Server

```bash
npm start
```

This command starts a local development server and opens the textbook in your browser. Most changes are reflected live without restarting the server.

### 3. Project Structure Overview

```
my-website/
├── docs/                    # Textbook content organized by modules and weeks
│   ├── module-1-robotic-nervous-system/
│   │   ├── week-1-introduction-to-physical-ai.md
│   │   ├── week-2-ros-2-fundamentals.md
│   │   └── week-3-python-agent-integration.md
│   ├── module-2-digital-twin/
│   │   ├── week-4-physics-simulation-in-gazebo.md
│   │   └── week-5-high-fidelity-rendering-in-unity.md
│   ├── module-3-ai-robot-brain/
│   │   ├── week-6-nvidia-isaac-sim.md
│   │   ├── week-7-isaac-ros-hardware-accelerated.md
│   │   └── week-8-isaac-sim-reinforcement-learning.md
│   └── module-4-vision-language-action/
│       ├── week-9-voice-to-action-with-openai-whisper.md
│       ├── week-10-cognitive-planning.md
│       └── week-11-13-capstone-autonomous-humanoid.md
├── src/
├── static/
├── docusaurus.config.js     # Docusaurus configuration
├── sidebars.js              # Navigation sidebar configuration
└── package.json
```

## Content Creation Workflow

### 1. Adding New Content

1. Create a new Markdown file in the appropriate module directory under `docs/`
2. Add proper Docusaurus frontmatter:

```markdown
---
title: Your Week Title
sidebar_label: Week Title
sidebar_position: X
---

# Your Week Title

Content goes here...
```

3. Update `sidebars.js` to include the new document in the navigation

### 2. Content Guidelines

- Follow the 4 Modules and 13 Weeks curriculum structure
- Include technical examples for ROS 2, Gazebo, NVIDIA Isaac, and VLA
- Ensure content is technically accurate and verifiable
- Use clear, educational language appropriate for students

## Building and Deployment

### 1. Build for Production

```bash
npm run build
```

This command generates static content into the `build/` directory and can be served using any static hosting service.

### 2. Local Preview of Build

```bash
npm run serve
```

This command serves the built static content in the `build/` directory for local preview.

### 3. GitHub Pages Deployment

The repository is configured with GitHub Actions to automatically deploy to GitHub Pages on pushes to the main branch. The workflow is defined in `.github/workflows/deploy.yml`.

## Configuration

### docusaurus.config.js

Key configuration options:

- `organizationName`: GitHub organization/username for deployment
- `projectName`: Repository name for GitHub Pages
- `baseUrl`: Base URL for the textbook site
- `favicon`: Path to favicon
- `themes`: Additional Docusaurus themes
- `plugins`: Additional Docusaurus plugins

### sidebars.js

Defines the navigation structure of the textbook following the 4 Modules and 13 Weeks curriculum:

- Module 1: The Robotic Nervous System (ROS 2) - 3 weeks
- Module 2: The Digital Twin (Gazebo & Unity) - 2 weeks
- Module 3: The AI-Robot Brain (NVIDIA Isaac™) - 3 weeks
- Module 4: Vision-Language-Action (VLA) - 4 weeks (including capstone)

## Technical Requirements

### Content Standards

- All ROS 2 commands, Gazebo workflows, NVIDIA Isaac SDK content, and robotics theory must be precise and verifiable
- Content must follow pedagogical progression from basic concepts to advanced integration
- Code examples must be valid and executable
- Technical accuracy verified against official documentation

### Performance Goals

- Fast loading times (<3 seconds)
- Mobile-friendly navigation
- Optimized for documentation browsing
- Proper accessibility compliance (WCAG 2.1 AA)