import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
    tutorialSidebar: [
        'intro',
        'index',
        'accessibility-statement',
        {
            type: 'category',
            label: 'Foundational Concepts',
            items: [
                '01-scope-boundaries',
                '02-ethical-dilemmas',
                '03-technical-concepts'
            ],
        },
        {
            type: 'category',
            label: 'Educational Applications',
            items: [
                '04-pedagogical-approaches',
                '05-education-levels',
                '06-implementation-guidance'
            ],
        },
        {
            type: 'category',
            label: 'Safety & Compliance',
            items: ['07-privacy-security'],
        },
        {
            type: 'category',
            label: 'Resources & Maintenance',
            items: ['versioning-strategy', 'update-procedures'],
        },
    ],
};

export default sidebars;