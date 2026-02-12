import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics in Education',
  tagline: 'Your comprehensive guide to integrating advanced robotics into learning',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://awais68.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-AI-Homanoid-Book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Awais68', // Usually your GitHub org/user name.
  projectName: 'physical-AI-Homanoid-Book', // Usually your repo name.

  onBrokenLinks: 'warn',
  trailingSlash: false,

  markdown: {
    format: 'detect',
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur', 'ur-PK', 'ar', 'es', 'fr', 'de', 'zh', 'hi', 'pt', 'ru', 'ja'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو (Urdu)',
        direction: 'rtl',
        htmlLang: 'ur',
      },
      'ur-PK': {
        label: 'Roman Urdu',
        direction: 'ltr',
        htmlLang: 'ur-PK',
      },
      ar: {
        label: 'العربية (Arabic)',
        direction: 'rtl',
        htmlLang: 'ar',
      },
      es: {
        label: 'Español (Spanish)',
        direction: 'ltr',
        htmlLang: 'es',
      },
      fr: {
        label: 'Français (French)',
        direction: 'ltr',
        htmlLang: 'fr',
      },
      de: {
        label: 'Deutsch (German)',
        direction: 'ltr',
        htmlLang: 'de',
      },
      zh: {
        label: '中文 (Chinese)',
        direction: 'ltr',
        htmlLang: 'zh-CN',
      },
      hi: {
        label: 'हिन्दी (Hindi)',
        direction: 'ltr',
        htmlLang: 'hi',
      },
      pt: {
        label: 'Português (Portuguese)',
        direction: 'ltr',
        htmlLang: 'pt',
      },
      ru: {
        label: 'Русский (Russian)',
        direction: 'ltr',
        htmlLang: 'ru',
      },
      ja: {
        label: '日本語 (Japanese)',
        direction: 'ltr',
        htmlLang: 'ja',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Awais68/physical-AI-Homanoid-Book/tree/main/frontend/',
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
            'https://github.com/Awais68/physical-AI-Homanoid-Book/tree/main/frontend/',
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
        src: 'img/er.jpeg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book Content',
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/Awais68/physical-AI-Homanoid-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/',
            },
            {
              label: 'Scope & Boundaries',
              to: '/docs/01-scope-boundaries',
            },
            {
              label: 'Ethical Framework',
              to: '/docs/02-ethical-dilemmas',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Technical Concepts',
              to: '/docs/03-technical-concepts',
            },
            {
              label: 'Pedagogical Approaches',
              to: '/docs/04-pedagogical-approaches',
            },
            {
              label: 'Implementation Guide',
              to: '/docs/06-implementation-guidance',
            },
          ],
        },
        {
          title: 'Connect',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/Awais68/physical-AI-Homanoid-Book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics in Education | Imagination Comes Into Reality With AS Developers`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
