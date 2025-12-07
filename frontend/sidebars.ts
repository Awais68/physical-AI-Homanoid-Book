import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Physical AI & Humanoid Robotics Book
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Book Content',
      items: [
        '01-scope-boundaries',
        '02-ethical-dilemmas',
        '03-technical-concepts',
        '04-pedagogical-approaches',
        '05-education-levels',
        '06-implementation-guidance',
        '07-privacy-security',
      ],
    },
    {
      type: 'category',
      label: 'Additional Resources',
      items: [
        'documentation-updates',
        'update-procedures',
        'versioning-strategy',
        'accessibility-statement',
      ],
    },
  ],
};

export default sidebars;
