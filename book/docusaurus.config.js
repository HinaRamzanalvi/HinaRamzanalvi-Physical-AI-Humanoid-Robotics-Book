// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Course',
  tagline: 'Dinosaurs are cool',
  favicon: 'img/favicon.ico',
  url: 'https://hina-ramzanalvi-physical-ai-humanoi.vercel.app',
  baseUrl: '/',
  trailingSlash: false,

  future: {
    v4: true,
  },

  // Additional configuration for the RagChatbot
  customFields: {
    apiUrl: process.env.API_URL || 'http://localhost:8000',
  },

  // Set the production url of your site here
  // Set the /<baseUrl>/ pathname under which your site is served

  // GitHub pages deployment config
  organizationName: 'HinaRamzanalvi', // your GitHub username
  projectName: 'HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book', // your repo name
  deploymentBranch: 'gh-pages',
  trailingSlash: false, // recommended for GitHub Pages

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book/tree/main/book/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book/tree/main/book/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],


  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics Course',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorial',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course Modules',
          items: [
            {
              label: 'Module 1: Robotic Nervous System (ROS 2)',
              to: '/docs/module1_ros2',
            },
            {
              label: 'Module 2: The Digital Twin (Gazebo & Unity)',
              to: '/docs/module2_digital_twin',
            },
            {
              label: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
              to: '/docs/module3_ai_robot_brain',
            },
            {
              label: 'Module 4: Vision-Language-Action (VLA)',
              to: '/docs/module4_vla',
            },
            {
              label: 'Module 5: RAG Chatbots for Robotics',
              to: '/docs/module5_rag_chatbots',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book',
            },
            {
              label: 'Documentation',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Connect',
          items: [
            {
              label: 'Report Issues',
              href: 'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book/issues',
            },
          ],
        },
      ],
      copyright: `Made by Hina Alvi | Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;