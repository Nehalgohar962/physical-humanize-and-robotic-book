import React from 'react';
const Footer = () => {
  // SVG Icons
  const GitHubIcon = () => (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor" style={{ marginRight: '8px', verticalAlign: 'middle' }}>
      <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
    </svg>
  );

  const LinkedInIcon = () => (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor" style={{ marginRight: '8px', verticalAlign: 'middle' }}>
      <path d="M20.447 20.452h-3.554v-5.569c0-1.328-.027-3.037-1.852-3.037-1.853 0-2.136 1.445-2.136 2.939v5.667H9.351V9h3.414v1.561h.046c.477-.9 1.637-1.85 3.37-1.85 3.601 0 4.267 2.37 4.267 5.455v6.286zM5.337 7.433c-1.144 0-2.063-.926-2.063-2.065 0-1.138.92-2.063 2.063-2.063 1.14 0 2.064.925 2.064 2.063 0 1.139-.925 2.065-2.064 2.065zm1.782 13.019H3.555V9h3.564v11.452zM22.225 0H1.771C.792 0 0 .774 0 1.729v20.542C0 23.227.792 24 1.771 24h20.451C23.2 24 24 23.227 24 22.271V1.729C24 .774 23.2 0 22.222 0h.003z"/>
    </svg>
  );

  const StackOverflowIcon = () => (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor" style={{ marginRight: '8px', verticalAlign: 'middle' }}>
      <path d="M21.5 1h-19C1.1 1 0 2.1 0 3.5v17c0 1.4 1.1 2.5 2.5 2.5h19c1.4 0 2.5-1.1 2.5-2.5v-17c0-1.4-1.1-2.5-2.5-2.5zM8.5 21h-3v-9h3v9zm1-12.5c-.8 0-1.5-.7-1.5-1.5s.7-1.5 1.5-1.5 1.5.7 1.5 1.5-.7 1.5-1.5 1.5zm10.5 12h-3v-5h3v5zm0-7h-3v-4h3v4z"/>
    </svg>
  );

  const DocusaurusIcon = () => (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor" style={{ marginRight: '8px', verticalAlign: 'middle' }}>
      <path d="M12 0c-6.627 0-12 5.373-12 12s5.373 12 12 12 12-5.373 12-12-5.373-12-12-12zm0 22c-5.514 0-10-4.486-10-10s4.486-10 10-10 10 4.486 10 10-4.486 10-10 10zm-2.29-8.71l-5.933-5.933c-.293-.293-.293-.768 0-1.06l.72-.72c.293-.293.768-.293 1.06 0l5.933 5.933c.293.293.293.768 0 1.06l-.72.72c-.293.293-.768.293-1.06 0zm8.58-8.58l-5.933-5.933c-.293-.293-.768-.293-1.06 0l-.72.72c-.293.293-.293.768 0 1.06l5.933 5.933c.293.293.768.293 1.06 0l.72-.72c.293-.293.293-.768 0-1.06z"/>
    </svg>
  );

  return (
    <footer className="footer">
      <div className="footer__container">
        <div className="footer__columns">
          {/* Column 1 */}
          <div className="footer__column">
            <div className="footer__section-title">Quick Links</div>
            <ul className="footer__list">
              <li className="footer__item"><a href="/docs">Modules</a></li>
              <li className="footer__item"><a href="/docs/about">About</a></li>
              <li className="footer__item"><a href="/docs/contact">Contact</a></li>
              <li className="footer__item"><a href="/chat">AI Assistant</a></li>
            </ul>
          </div>

          {/* Column 2 */}
          <div className="footer__column">
            <div className="footer__section-title">Connect</div>
            <ul className="footer__list">
              <li className="footer__item"><a href="https://github.com/Nehalgohar962/physical-humanize-and-robotic-book" target="_blank">GitHub <GitHubIcon /></a></li>
              <li className="footer__item"><a href="https://www.linkedin.com/in/nehal-gohar-654608295/" target="_blank">LinkedIn <LinkedInIcon /></a></li>
              <li className="footer__item"><a href="https://stackoverflow.com/questions/tagged/docusaurus" target="_blank">Stack Overflow <StackOverflowIcon /></a></li>
            </ul>
          </div>

          {/* Column 3 */}
          <div className="footer__column">
            <div className="footer__section-title">More</div>
            <ul className="footer__list">
              <li className="footer__item"><a href="https://docusaurus.io/" target="_blank">Docusaurus <DocusaurusIcon /></a></li>
            </ul>
          </div>
        </div>

        <div className="footer__bottom">
          <div className="footer__copyright">
            © 2026 Physical AI & Humanoid Robotics Book — Nehal Gohar ♥ All rights reserved.
          </div>
        </div>
      </div>
    </footer>
  );
};
export default Footer;