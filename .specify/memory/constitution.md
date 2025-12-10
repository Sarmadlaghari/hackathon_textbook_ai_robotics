<!--
SYNC IMPACT REPORT
Version change: N/A (initial creation) → 1.0.0
Added sections: All sections (Core Principles, Constraints and Standards, Success Criteria, Governance)
Removed sections: None
Modified principles: N/A (initial creation)
Templates requiring updates:
  - ✅ plan-template.md (already has Constitution Check section that will incorporate new principles)
  - ✅ spec-template.md (no specific constitution references to update)
  - ✅ tasks-template.md (no specific constitution references to update)
Follow-up TODOs: None
-->
# AI-Native Textbook for Physical AI & Humanoid Robotics Constitution

## Core Principles

### Textbook Creation Focus
All generation must strictly produce high-quality, technically accurate textbook content and structure.

### Curriculum Structure Adherence
The textbook must follow the exact 4 Modules and 13-Week curriculum structure (ROS 2, Gazebo, NVIDIA Isaac, VLA).

### Technical Stack Compliance
Book platform: Docusaurus; AI agent tools: Claude Code and SpecifyPlus; Deployment correctness: All generated project files must be fully valid for GitHub Pages deployment with proper Docusaurus configuration.

### Technical Accuracy Requirement
All ROS 2 commands, Gazebo workflows, NVIDIA Isaac SDK content, and robotics theory must be precise and verifiable.

### Content Modularity
All content must be organized into logically separated Markdown files inside the correct Docusaurus directory structure.

### Content Completeness
All 13 weeks and 4 modules must be drafted with full coverage.

## Constraints and Standards

Platform constraint: Only Docusaurus directory conventions allowed (docs/, sidebars.js, versioning, etc.).
Tool constraint: All development and generation assumed to be executed via Claude Code + SpecifyPlus.
Structure constraint: Book content must remain within the 4-Module, 13-Week outline—no deviations.
Deployment constraint: Output must always be directly deployable to GitHub Pages.

## Success Criteria

All modules and weeks fully drafted.
Docusaurus project builds without errors.
GitHub Pages deployment works without modification.
Content remains technically accurate across all robotics frameworks.

## Governance

All development must strictly adhere to the defined curriculum structure and technical stack. Any deviations from the established patterns must be documented and approved. Code reviews must verify compliance with Docusaurus conventions, deployment requirements, and technical accuracy standards.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
