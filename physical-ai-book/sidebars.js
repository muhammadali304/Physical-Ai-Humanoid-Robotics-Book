/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
module.exports = {
  // Manual sidebar configuration with proper capitalization
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Setup',
          items: [
            'setup/ubuntu-preparation',
            'setup/ubuntu-installation',
            'setup/ros2-installation',
            'setup/ros2-setup',
            'setup/ros2-environment-setup',
            'setup/ros2-workspace-structure',
            'setup/running-examples',
            'setup/performance-benchmark',
            'setup/validation-testing'
          ],
        },
        {
          type: 'category',
          label: 'ROS 2 Fundamentals',
          items: [
            'ros2-fundamentals/nodes-topics',
            'ros2-fundamentals/packages',
            'ros2-fundamentals/services-actions'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Simulation',
          items: [
            'simulation/gazebo-installation',
            'simulation/gazebo-basics',
            'simulation/urdf-modeling',
            'simulation/custom-world-creation',
            'simulation/sensor-visualization-rviz2',
            'simulation/sensor-data-verification',
            'simulation/physics-performance-optimization',
            'simulation/fps-validation-process',
            'simulation/navigation-testing-environment',
            'simulation/simulation-troubleshooting',
            'simulation/unity-integration'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Isaac Platform',
          items: [
            'isaac-platform/isaac-ros-installation',
            'isaac-platform/isaac-ros-perception',
            'isaac-platform/isaac-ros-perception-setup',
            'isaac-platform/nav2-setup-guide',
            'isaac-platform/vsland-pipeline-configuration',
            'isaac-platform/robot-localization-amcl',
            'isaac-platform/costmap-configuration',
            'isaac-platform/path-planning-algorithms',
            'isaac-platform/path-execution-controller',
            'isaac-platform/perception-navigation-integration',
            'isaac-platform/navigation',
            'isaac-platform/navigation-validation-guide',
            'isaac-platform/goal-sending-interface',
            'isaac-platform/perception-performance-benchmark',
            'isaac-platform/isaac-sim-intro',
            'isaac-platform/isaac-sim-rl-setup-guide',
            'isaac-platform/robot-manipulation-rl-example',
            'isaac-platform/llm-integration-guide',
            'isaac-platform/voice-command-processing-system',
            'isaac-platform/sim-to-real-transfer-jetson-guide',
            'isaac-platform/edge-device-optimization-techniques'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'VLA',
          items: [
            'vla/integration',
            'vla/llm-planning',
            'vla/voice-commands'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsed: true,
      items: [
        'capstone/capstone-project-integration-guide',
        'capstone/autonomous-humanoid'
      ],
    },
    {
      type: 'category',
      label: 'Appendix',
      collapsed: true,
      items: [
        'appendix/troubleshooting',
        'appendix/troubleshooting-ros2',
        'appendix/hardware-guide',
        'appendix/security-best-practices',
        'appendix/assessment-methods',
        'appendix/external-service-failure-handling',
        'accessibility-statement'
      ],
    },
  ],
};