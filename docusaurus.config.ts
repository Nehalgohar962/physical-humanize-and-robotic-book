import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A book on the intersection of Physical AI and Humanoid Robotics',
  favicon: 'img/robotics-hero.jpg',

  future: { v4: true },

  url: '/', 
  baseUrl: '/', 

  organizationName: 'Nehal Gohar',
  projectName: 'physical-humanize-and-robotic-book',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'throw',

  i18n: { defaultLocale: 'en', locales: ['en'] },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/Nehalgohar962/physical-humanize-and-robotic-book/tree/main/',
        },
        // Blog removed completely
        theme: { customCss: './src/css/custom.css' },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: { src: 'img/robotics-hero.jpg' },
      items: [
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Modules' },
        { to: '/docs/about', label: 'About', position: 'left' },
        { to: '/docs/contact', label: 'Contact', position: 'left' },
        { href: 'https://github.com/Nehalgohar962/physical-humanize-and-robotic-book', label: 'GitHub', position: 'right' },
        { href: 'https://www.linkedin.com/in/nehal-gohar-654608295/', label: 'LinkedIn', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Quick Links',
          items: [
            { label: 'Modules', to: '/docs' },
            { label: 'About', to: '/docs/about' },
            { label: 'Contact', to: '/docs/contact' },
          ],
        },
        {
          title: 'Connect',
          items: [
            { label: 'GitHub', href: 'https://github.com/Nehalgohar962/physical-humanize-and-robotic-book' },
            { label: 'LinkedIn', href: 'https://www.linkedin.com/in/nehal-gohar-654608295/' },
            { label: 'Stack Overflow', href: 'https://stackoverflow.com/questions/tagged/docusaurus' },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'Docusaurus', href: 'https://docusaurus.io/' },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Nehal Gohar ♥ All rights reserved.`,
    },
    prism: { theme: prismThemes.github, darkTheme: prismThemes.dracula },
  } satisfies Preset.ThemeConfig,
};

export default config;


