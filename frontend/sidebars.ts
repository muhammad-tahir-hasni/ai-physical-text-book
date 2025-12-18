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
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      collapsed: false,
      items: [
        'module-1/ros2-architecture',
        'module-1/building-ros2-apps',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation & URDF',
      collapsed: false,
      items: [
        'module-2/gazebo-fundamentals',
        'module-2/urdf-robot-description',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Computer Vision',
      collapsed: false,
      items: [
        'module-3/isaac-sim-intro',
        'module-3/ai-perception',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      collapsed: false,
      items: [
        'module-4/llm-cognitive-planning',
        'module-4/voice-to-action',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Basics',
      collapsed: true,
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/markdown-features',
        'tutorial-basics/congratulations',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      collapsed: true,
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
    'capstone-guidance',
    'debugging-guide',
    'bibliography',
  ],
};

export default sidebars;
