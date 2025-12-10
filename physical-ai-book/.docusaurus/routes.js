import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '828'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '688'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'b63'),
            routes: [
              {
                path: '/docs/accessibility-statement',
                component: ComponentCreator('/docs/accessibility-statement', '1ea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendix/assessment-methods',
                component: ComponentCreator('/docs/appendix/assessment-methods', '91b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendix/external-service-failure-handling',
                component: ComponentCreator('/docs/appendix/external-service-failure-handling', 'dfe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendix/hardware-guide',
                component: ComponentCreator('/docs/appendix/hardware-guide', 'dc0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendix/security-best-practices',
                component: ComponentCreator('/docs/appendix/security-best-practices', '65b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendix/troubleshooting',
                component: ComponentCreator('/docs/appendix/troubleshooting', '3ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/appendix/troubleshooting-ros2',
                component: ComponentCreator('/docs/appendix/troubleshooting-ros2', 'd1c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/autonomous-humanoid',
                component: ComponentCreator('/docs/capstone/autonomous-humanoid', 'e47'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/capstone-project-integration-guide',
                component: ComponentCreator('/docs/capstone/capstone-project-integration-guide', '77c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/costmap-configuration',
                component: ComponentCreator('/docs/isaac-platform/costmap-configuration', 'bd3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/edge-device-optimization-techniques',
                component: ComponentCreator('/docs/isaac-platform/edge-device-optimization-techniques', '4a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/goal-sending-interface',
                component: ComponentCreator('/docs/isaac-platform/goal-sending-interface', '7bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/isaac-ros-installation',
                component: ComponentCreator('/docs/isaac-platform/isaac-ros-installation', '7af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/isaac-ros-perception',
                component: ComponentCreator('/docs/isaac-platform/isaac-ros-perception', 'c26'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/isaac-ros-perception-setup',
                component: ComponentCreator('/docs/isaac-platform/isaac-ros-perception-setup', '51a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/isaac-sim-intro',
                component: ComponentCreator('/docs/isaac-platform/isaac-sim-intro', 'e90'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/isaac-sim-rl-setup-guide',
                component: ComponentCreator('/docs/isaac-platform/isaac-sim-rl-setup-guide', '761'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/llm-integration-guide',
                component: ComponentCreator('/docs/isaac-platform/llm-integration-guide', '28c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/nav2-setup-guide',
                component: ComponentCreator('/docs/isaac-platform/nav2-setup-guide', '744'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/navigation',
                component: ComponentCreator('/docs/isaac-platform/navigation', 'cea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/navigation-validation-guide',
                component: ComponentCreator('/docs/isaac-platform/navigation-validation-guide', 'e44'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/path-execution-controller',
                component: ComponentCreator('/docs/isaac-platform/path-execution-controller', '911'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/path-planning-algorithms',
                component: ComponentCreator('/docs/isaac-platform/path-planning-algorithms', 'd1a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/perception-navigation-integration',
                component: ComponentCreator('/docs/isaac-platform/perception-navigation-integration', '0b9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/perception-performance-benchmark',
                component: ComponentCreator('/docs/isaac-platform/perception-performance-benchmark', 'eae'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/robot-localization-amcl',
                component: ComponentCreator('/docs/isaac-platform/robot-localization-amcl', '5d4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/robot-manipulation-rl-example',
                component: ComponentCreator('/docs/isaac-platform/robot-manipulation-rl-example', '208'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/sim-to-real-transfer-jetson-guide',
                component: ComponentCreator('/docs/isaac-platform/sim-to-real-transfer-jetson-guide', '8b1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/voice-command-processing-system',
                component: ComponentCreator('/docs/isaac-platform/voice-command-processing-system', '438'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/isaac-platform/vsland-pipeline-configuration',
                component: ComponentCreator('/docs/isaac-platform/vsland-pipeline-configuration', '38a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-fundamentals/nodes-topics',
                component: ComponentCreator('/docs/ros2-fundamentals/nodes-topics', 'f36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-fundamentals/packages',
                component: ComponentCreator('/docs/ros2-fundamentals/packages', '3ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/ros2-fundamentals/services-actions',
                component: ComponentCreator('/docs/ros2-fundamentals/services-actions', 'b5e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/performance-benchmark',
                component: ComponentCreator('/docs/setup/performance-benchmark', '310'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/ros2-environment-setup',
                component: ComponentCreator('/docs/setup/ros2-environment-setup', '9f5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/ros2-installation',
                component: ComponentCreator('/docs/setup/ros2-installation', 'c8a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/ros2-setup',
                component: ComponentCreator('/docs/setup/ros2-setup', '904'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/ros2-workspace-structure',
                component: ComponentCreator('/docs/setup/ros2-workspace-structure', '37a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/running-examples',
                component: ComponentCreator('/docs/setup/running-examples', '2ce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/ubuntu-installation',
                component: ComponentCreator('/docs/setup/ubuntu-installation', '5c1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/ubuntu-preparation',
                component: ComponentCreator('/docs/setup/ubuntu-preparation', 'f77'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/setup/validation-testing',
                component: ComponentCreator('/docs/setup/validation-testing', 'd3c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/custom-world-creation',
                component: ComponentCreator('/docs/simulation/custom-world-creation', 'ed2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/fps-validation-process',
                component: ComponentCreator('/docs/simulation/fps-validation-process', '089'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/gazebo-basics',
                component: ComponentCreator('/docs/simulation/gazebo-basics', 'f1a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/gazebo-installation',
                component: ComponentCreator('/docs/simulation/gazebo-installation', '7e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/navigation-testing-environment',
                component: ComponentCreator('/docs/simulation/navigation-testing-environment', 'ca1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/physics-performance-optimization',
                component: ComponentCreator('/docs/simulation/physics-performance-optimization', '88f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/sensor-data-verification',
                component: ComponentCreator('/docs/simulation/sensor-data-verification', 'c80'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/sensor-visualization-rviz2',
                component: ComponentCreator('/docs/simulation/sensor-visualization-rviz2', '2ec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/simulation-troubleshooting',
                component: ComponentCreator('/docs/simulation/simulation-troubleshooting', '253'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/unity-integration',
                component: ComponentCreator('/docs/simulation/unity-integration', '6da'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/simulation/urdf-modeling',
                component: ComponentCreator('/docs/simulation/urdf-modeling', 'aec'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/integration',
                component: ComponentCreator('/docs/vla/integration', '264'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/llm-planning',
                component: ComponentCreator('/docs/vla/llm-planning', '3b8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/vla/voice-commands',
                component: ComponentCreator('/docs/vla/voice-commands', 'b3e'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
