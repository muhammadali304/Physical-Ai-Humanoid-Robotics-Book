import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/./docs',
    component: ComponentCreator('/./docs', '853'),
    routes: [
      {
        path: '/./docs',
        component: ComponentCreator('/./docs', '775'),
        routes: [
          {
            path: '/./docs',
            component: ComponentCreator('/./docs', '254'),
            routes: [
              {
                path: '/./docs/accessibility-statement',
                component: ComponentCreator('/./docs/accessibility-statement', 'd3b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/appendix/assessment-methods',
                component: ComponentCreator('/./docs/appendix/assessment-methods', 'be4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/appendix/external-service-failure-handling',
                component: ComponentCreator('/./docs/appendix/external-service-failure-handling', 'aaf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/appendix/hardware-guide',
                component: ComponentCreator('/./docs/appendix/hardware-guide', 'd68'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/appendix/security-best-practices',
                component: ComponentCreator('/./docs/appendix/security-best-practices', '478'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/appendix/troubleshooting',
                component: ComponentCreator('/./docs/appendix/troubleshooting', 'fb6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/appendix/troubleshooting-ros2',
                component: ComponentCreator('/./docs/appendix/troubleshooting-ros2', 'a96'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/capstone/autonomous-humanoid',
                component: ComponentCreator('/./docs/capstone/autonomous-humanoid', '71a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/capstone/capstone-project-integration-guide',
                component: ComponentCreator('/./docs/capstone/capstone-project-integration-guide', 'f0f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/intro',
                component: ComponentCreator('/./docs/intro', 'e26'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/costmap-configuration',
                component: ComponentCreator('/./docs/isaac-platform/costmap-configuration', '226'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/edge-device-optimization-techniques',
                component: ComponentCreator('/./docs/isaac-platform/edge-device-optimization-techniques', 'e70'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/goal-sending-interface',
                component: ComponentCreator('/./docs/isaac-platform/goal-sending-interface', '10d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/isaac-ros-installation',
                component: ComponentCreator('/./docs/isaac-platform/isaac-ros-installation', '0d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/isaac-ros-perception',
                component: ComponentCreator('/./docs/isaac-platform/isaac-ros-perception', '249'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/isaac-ros-perception-setup',
                component: ComponentCreator('/./docs/isaac-platform/isaac-ros-perception-setup', '972'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/isaac-sim-intro',
                component: ComponentCreator('/./docs/isaac-platform/isaac-sim-intro', '9e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/isaac-sim-rl-setup-guide',
                component: ComponentCreator('/./docs/isaac-platform/isaac-sim-rl-setup-guide', '08c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/llm-integration-guide',
                component: ComponentCreator('/./docs/isaac-platform/llm-integration-guide', '3c7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/nav2-setup-guide',
                component: ComponentCreator('/./docs/isaac-platform/nav2-setup-guide', 'b22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/navigation',
                component: ComponentCreator('/./docs/isaac-platform/navigation', '054'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/navigation-validation-guide',
                component: ComponentCreator('/./docs/isaac-platform/navigation-validation-guide', '1ed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/path-execution-controller',
                component: ComponentCreator('/./docs/isaac-platform/path-execution-controller', '4af'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/path-planning-algorithms',
                component: ComponentCreator('/./docs/isaac-platform/path-planning-algorithms', '525'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/perception-navigation-integration',
                component: ComponentCreator('/./docs/isaac-platform/perception-navigation-integration', '02b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/perception-performance-benchmark',
                component: ComponentCreator('/./docs/isaac-platform/perception-performance-benchmark', '762'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/robot-localization-amcl',
                component: ComponentCreator('/./docs/isaac-platform/robot-localization-amcl', 'b9c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/robot-manipulation-rl-example',
                component: ComponentCreator('/./docs/isaac-platform/robot-manipulation-rl-example', '624'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/sim-to-real-transfer-jetson-guide',
                component: ComponentCreator('/./docs/isaac-platform/sim-to-real-transfer-jetson-guide', '2ce'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/voice-command-processing-system',
                component: ComponentCreator('/./docs/isaac-platform/voice-command-processing-system', '9d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/isaac-platform/vsland-pipeline-configuration',
                component: ComponentCreator('/./docs/isaac-platform/vsland-pipeline-configuration', '669'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/ros2-fundamentals/nodes-topics',
                component: ComponentCreator('/./docs/ros2-fundamentals/nodes-topics', 'b96'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/ros2-fundamentals/packages',
                component: ComponentCreator('/./docs/ros2-fundamentals/packages', 'ac6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/ros2-fundamentals/services-actions',
                component: ComponentCreator('/./docs/ros2-fundamentals/services-actions', '704'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/performance-benchmark',
                component: ComponentCreator('/./docs/setup/performance-benchmark', '1f2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/ros2-environment-setup',
                component: ComponentCreator('/./docs/setup/ros2-environment-setup', '57c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/ros2-installation',
                component: ComponentCreator('/./docs/setup/ros2-installation', '3f6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/ros2-setup',
                component: ComponentCreator('/./docs/setup/ros2-setup', '613'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/ros2-workspace-structure',
                component: ComponentCreator('/./docs/setup/ros2-workspace-structure', '2f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/running-examples',
                component: ComponentCreator('/./docs/setup/running-examples', '0f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/ubuntu-installation',
                component: ComponentCreator('/./docs/setup/ubuntu-installation', '47e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/ubuntu-preparation',
                component: ComponentCreator('/./docs/setup/ubuntu-preparation', '88f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/setup/validation-testing',
                component: ComponentCreator('/./docs/setup/validation-testing', '9a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/custom-world-creation',
                component: ComponentCreator('/./docs/simulation/custom-world-creation', '50a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/fps-validation-process',
                component: ComponentCreator('/./docs/simulation/fps-validation-process', '8bb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/gazebo-basics',
                component: ComponentCreator('/./docs/simulation/gazebo-basics', 'd1f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/gazebo-installation',
                component: ComponentCreator('/./docs/simulation/gazebo-installation', 'e22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/navigation-testing-environment',
                component: ComponentCreator('/./docs/simulation/navigation-testing-environment', '687'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/physics-performance-optimization',
                component: ComponentCreator('/./docs/simulation/physics-performance-optimization', '20a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/sensor-data-verification',
                component: ComponentCreator('/./docs/simulation/sensor-data-verification', 'd0b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/sensor-visualization-rviz2',
                component: ComponentCreator('/./docs/simulation/sensor-visualization-rviz2', '395'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/simulation-troubleshooting',
                component: ComponentCreator('/./docs/simulation/simulation-troubleshooting', '466'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/unity-integration',
                component: ComponentCreator('/./docs/simulation/unity-integration', '178'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/simulation/urdf-modeling',
                component: ComponentCreator('/./docs/simulation/urdf-modeling', '6cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/vla/integration',
                component: ComponentCreator('/./docs/vla/integration', '475'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/vla/llm-planning',
                component: ComponentCreator('/./docs/vla/llm-planning', 'c61'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/./docs/vla/voice-commands',
                component: ComponentCreator('/./docs/vla/voice-commands', '18f'),
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
    path: '/./',
    component: ComponentCreator('/./', '309'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
