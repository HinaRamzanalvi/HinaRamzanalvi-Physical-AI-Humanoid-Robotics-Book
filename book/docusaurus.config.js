// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Course',
  tagline: 'Dinosaurs are cool',
  favicon: 'img/favicon.ico',

  url: 'https://hinaramzanalvi.github.io',
  baseUrl: '/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book/',
  trailingSlash: false,

  future: {
    v4: true,
  },

  customFields: {
    apiUrl: process.env.API_URL || 'http://localhost:8000',
  },

  // GitHub values yahan rehne do (Vercel par ignore ho jate hain)
  organizationName: 'HinaRamzanalvi',
  projectName: 'HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book',

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
          editUrl:
            'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book/tree/main/book/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book/tree/main/book/',
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
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href:
            'https://github.com/HinaRamzanalvi/HinaRamzanalvi-Physical-AI-Humanoid-Robotics-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Made by Hina Alvi | © ${new Date().getFullYear()}`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;

