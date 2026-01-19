import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Course Modules',
      items: [
        'module1',
        'module2',
        'module3',
        'module4',
        'module5',
        'module6',
      ],
    },
    // About aur Contact ko sidebar me alag rakha
    'about',
    'contact',
  ],
};

export default sidebars;
