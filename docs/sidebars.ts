import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for MCP Guide and tutorials
  tutorialSidebar: [
    'physical-ai-robotics/introduction',
    'physical-ai-robotics/ros2-robotic-nervous-system',
    {
      type: 'category',
      label: 'Module Tutorials',
      items: [
        'physical-ai-robotics/module-1-ros2/overview',
        'physical-ai-robotics/module-2-digital-twin/overview',
        'physical-ai-robotics/module-3-isaac-brain/overview',
        'physical-ai-robotics/module-4-vla/overview',
      ],
    },
  ],

  // Simplified sidebar for Physical AI Robotics documentation
  physicalAIRoboticsSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        'physical-ai-robotics/introduction',
        'physical-ai-robotics/introduction-to-physical-ai',
        'physical-ai-robotics/ros2-robotic-nervous-system',
        'physical-ai-robotics/module-1-ros2/overview',
        'physical-ai-robotics/digital-twin-simulation',
        'physical-ai-robotics/module-2-digital-twin/overview',
        'physical-ai-robotics/ai-robot-brain-isaac',
        'physical-ai-robotics/module-3-isaac-brain/overview',
        'physical-ai-robotics/vision-language-action',
        'physical-ai-robotics/module-4-vla/overview',
        {
          type: 'category',
          label: 'Capstone Projects',
          items: [
            'physical-ai-robotics/capstone-autonomous-humanoid',
            'physical-ai-robotics/capstone-the-autonomous-humanoid',
          ],
        },
        {
          type: 'category',
          label: 'Resources',
          items: [
            'physical-ai-robotics/hardware-requirements',
            'physical-ai-robotics/cloud-vs-on-premise-lab-setup',
            'physical-ai-robotics/lab-architecture',
            'physical-ai-robotics/weekly-breakdown',
            'physical-ai-robotics/assessments',
            'physical-ai-robotics/the-economy-jetson-student-kit',
          ],
        },
      ],
    },
  ],
};

export default sidebars;