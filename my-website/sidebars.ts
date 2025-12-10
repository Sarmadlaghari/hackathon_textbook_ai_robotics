import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar structure for the AI-Native Textbook following 4 Modules and 13 Weeks curriculum
  textbookSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      collapsible: true,
      collapsed: false,
      items: [
        'quickstart',
      ],
    },
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1-robotic-nervous-system/week-1-introduction-to-physical-ai',
        'module-1-robotic-nervous-system/week-2-ros-2-fundamentals',
        'module-1-robotic-nervous-system/week-3-python-agent-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 — Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2-digital-twin/week-4-physics-simulation-in-gazebo',
        'module-2-digital-twin/week-5-high-fidelity-rendering-in-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 — The AI-Robot Brain (NVIDIA Isaac™)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3-ai-robot-brain/week-6-nvidia-isaac-sim',
        'module-3-ai-robot-brain/week-7-isaac-ros-hardware-accelerated',
        'module-3-ai-robot-brain/week-8-isaac-sim-reinforcement-learning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4-vision-language-action/week-9-voice-to-action-with-openai-whisper',
        'module-4-vision-language-action/week-10-cognitive-planning',
        'module-4-vision-language-action/week-11-13-capstone-autonomous-humanoid',
      ],
    },
  ],
};

export default sidebars;
