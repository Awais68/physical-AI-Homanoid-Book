import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
    title: 'Physical AI & Humanoid Robotics in Education',
    tagline: 'A comprehensive guide to integrating advanced robotics into educational contexts',
    favicon: 'img/er.jpeg',

    // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
    future: {
        v4: true, // Improve compatibility with the upcoming Docusaurus v4
    },

    // Set the production url of your site here
    url: 'http://localhost:3000',
    baseUrl: '/',

    organizationName: 'your-organization',
    projectName: 'physical-ai-humanoid-robotics-book',

    onBrokenLinks: 'warn',
    onBrokenMarkdownLinks: 'warn',
    markdown: {
        format: 'mdx',
        mermaid: true,
        // Migrate deprecated option
        hooks: {
            onBrokenMarkdownLinks: 'warn',
        },
    },

    i18n: {
        defaultLocale: 'en',
        locales: ['en'],
    },

    presets: [
        [
            'classic',
            {
                docs: {
                    sidebarPath: './sidebars.ts',
                    editUrl:
                        'https://github.com/your-organization/physical-ai-humanoid-robotics-book/tree/main/',
                    showLastUpdateAuthor: true,
                    showLastUpdateTime: true,
                },
                blog: {
                    showReadingTime: true,
                    editUrl:
                        'https://github.com/your-organization/physical-ai-humanoid-robotics-book/tree/main/',
                },
                theme: {
                    customCss: './src/css/custom.css',
                },
            } satisfies Preset.Options,
        ],
    ],

    themeConfig: {
        image: 'img/docusaurus-social-card.jpg', // Path to image used for social sharing (e.g., when links are shared on social media)
        navbar: {
            title: 'Physical AI & Humanoid Robotics',
            logo: {
                alt: 'Physical AI & Humanoid Robotics in Education Logo',
                src: 'img/er.jpeg',
                srcDark: 'img/er.jpeg', // Add dark mode version if available
            },
            items: [
                {
                    type: 'docSidebar',
                    sidebarId: 'tutorialSidebar',
                    position: 'left',
                    label: 'Book Content',
                },
                {
                    href: 'https://github.com/your-organization/physical-ai-humanoid-robotics-book',
                    label: 'GitHub',
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
                            label: 'Getting Started',
                            to: '/docs/intro',
                        },
                        {
                            label: 'Book Index',
                            to: '/docs/index',
                        },
                    ],
                },
                {
                    title: 'More',
                    items: [
                        {
                            label: 'GitHub',
                            href: 'https://github.com/your-organization/physical-ai-humanoid-robotics-book',
                        },
                    ],
                },
            ],
            copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics in Education. Built with Docusaurus.`,
        },
        prism: {
            theme: prismThemes.github,
            darkTheme: prismThemes.dracula,
        },
    } satisfies Preset.ThemeConfig,
};

export default config;
