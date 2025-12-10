// import React from 'react';
// import Link from '@docusaurus/Link';
// import Layout from '@theme/Layout';
// import '../css/home.css'; // Updated CSS file

// export default function Home() {
//   return (
//     <Layout
//       title="Physical AI & Humanoid Robotics"
//       description="A book on the intersection of Physical AI and Humanoid Robotics"
//     >
//       {/* HERO SECTION */}
//       <header className="hero-banner">
//         <div className="hero-content">
//           <h1>Physical AI & Humanoid Robotics</h1>
//           <p>A practical guide to the future of intelligent physical machines</p>
//           <Link className="start-button" to="/modules">
//             Start Reading Book ➜
//           </Link>
//         </div>
//       </header>

//       {/* FEATURE CARDS SECTION */}
//       <section className="feature-section">
//         <h2 className="feature-title">Key Highlights of This Book</h2>
//         <div className="feature-container">
//           {/* CARD 1 */}
//           <div className="feature-card">
//             <h3>🤖 What You Will Learn</h3>
//             <p>
//               Core foundations of Physical AI, robotics, biomechanics, motion systems, and intelligent machines.
//             </p>
//           </div>

//           {/* CARD 2 */}
//           <div className="feature-card">
//             <h3>📘 Full Module Breakdown</h3>
//             <p>
//               Complete structured modules from basics to advanced humanoid robotics engineering.
//             </p>
//           </div>

//           {/* CARD 3 */}
//           <div className="feature-card">
//             <h3>🧠 AI + Robotics Insights</h3>
//             <p>
//               Latest industry knowledge connecting AI, design, movement, and human-robot interaction.
//             </p>
//           </div>
//         </div>
//       </section>

      
//     </Layout>
//   );
// }




// src/pages/Home.tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import BookChat from '../components/BookChat';
import '../css/home.css';
import '../css/chat.css';
import '../css/modules.css';
import '../css/custom.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A book on the intersection of Physical AI and Humanoid Robotics"
    >
      {/* HERO SECTION */}
      <header className="hero-banner">
        <div className="hero-content">
          <h1>Physical AI & Humanoid Robotics</h1>
          <p>A practical guide to the future of intelligent physical machines</p>
          <Link className="start-button" to="/modules">
            Start Reading Book ➜
          </Link>
        </div>
      </header>

      {/* FEATURE CARDS SECTION */}
      <section className="feature-section">
        <h2 className="feature-title">Key Highlights of This Book</h2>
        <div className="feature-container">
          {/* CARD 1 */}
          <div className="feature-card">
            <h3>🤖 What You Will Learn</h3>
            <p>
              Core foundations of Physical AI, robotics, biomechanics, motion systems, and intelligent machines.
            </p>
          </div>

          {/* CARD 2 */}
          <div className="feature-card">
            <h3>📘 Full Module Breakdown</h3>
            <p>
              Complete structured modules from basics to advanced humanoid robotics engineering.
            </p>
          </div>

          {/* CARD 3 */}
          <div className="feature-card">
            <h3>🧠 AI + Robotics Insights</h3>
            <p>
              Latest industry knowledge connecting AI, design, movement, and human-robot interaction.
            </p>
          </div>
        </div>
      </section>

      {/* CHAT BOX FLOATING */}
      <BookChat /> 
    </Layout>
  );
}
