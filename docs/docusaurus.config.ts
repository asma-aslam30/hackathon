import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',

  // Head tags for custom fonts
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;500;600;700&display=swap',
      },
    },
  ],
  tagline: 'Comprehensive Guide to Building Intelligent Physical Systems',
  favicon: 'https://www.shutterstock.com/image-vector/robotic-hand-manipulator-symbol-icon-600nw-2523988227.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://asma-aslam30.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'asma-aslam30', // Usually your GitHub org/user name.
  projectName: 'hackathon', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenAnchors: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        docsRouteBasePath: '/docs',
        blogRouteBasePath: '/blog',
        // Disable keyboard shortcut to avoid the toLowerCase error
        searchBarShortcut: false,
        searchBarShortcutHint: false,
      },
    ],
  ],
  scripts: [
    {
      src: '/js/interactive-ui.js',
      async: true,
      defer: true,
    },
  ],
  staticDirectories: ['static', 'public'],
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/asma-aslam30/hackathon/tree/main/docs/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Physical AI & Robotics Logo',
        src: 'img/robot-icon.svg', // We'll create this
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'MCP Guide',
        },
        {
          type: 'docSidebar',
          sidebarId: 'physicalAIRoboticsSidebar',
          position: 'left',
          label: 'Physical AI & Robotics',
        },
        { to: '/chatbot', label: 'AI Chatbot', position: 'left' },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          type: 'custom-translate',
          position: 'right',
        },
        {
          href: 'https://github.com/asma-aslam30/hackathon',
          label: 'GitHub',
          position: 'right',
        },
        {
          to: '/profile',
          label: 'Profile',
          position: 'right',
        },
        {
          to: '/login',
          label: 'Login',
          position: 'right',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Physical AI & Robotics',
              to: '/docs/physical-ai-robotics/introduction',
            },
            {
              label: 'AI Robot Brain (Isaac)',
              to: '/docs/physical-ai-robotics/ai-robot-brain-isaac',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'X',
              href: 'https://x.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/asma-aslam30/hackathon',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    // Add custom styling
    announcementBar: {
      id: 'announcement-bar',
      content: '🏗️ <b>Under active development</b>: We\'re continuously adding new content about Physical AI & MCP integration',
      backgroundColor: '#2962ff',
      textColor: '#ffffff',
      isCloseable: false,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;