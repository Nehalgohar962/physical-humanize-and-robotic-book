// src/pages/Home.tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import BookChat from '../components/BookChat';
import '../css/custom.css';

export default function Home() {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="A book on the intersection of Physical AI and Humanoid Robotics"
    >
      {/* HERO SECTION */}
      <header className="hero-banner-image">
        <div className="hero-content">
          <h1>Physical AI & Humanoid Robotics</h1>
          <p>A practical guide to the future of intelligent physical machines, focusing on humanoid robotics, AI-powered movement, sensing, control systems, and the technologies shaping next-generation robots.</p>
          <Link className="start-button" to="/modules">
            Start Reading Book ➜
          </Link>
        </div>
      </header>

      {/* HACKATHON TECH STACK PROGRESS SECTION */}
      <section className="tech-stack-section">
        <h2 className="tech-stack-title">Hackathon Tech Stack</h2>
        <p className="tech-stack-subtitle">Technologies and proficiency levels</p>
        <div className="progress-bars-container">
          <div className="progress-item">
            <div className="progress-header">
              <span className="tech-name">Python (ROS 2, AI Logic)</span>
              <span className="tech-percent">85%</span>
            </div>
            <div className="progress-bar">
              <div className="progress-fill python-fill" style={{ width: '85%', '--target-width': '85%' } as React.CSSProperties}><span>85%</span></div>
            </div>
          </div>

          <div className="progress-item">
            <div className="progress-header">
              <span className="tech-name">JavaScript / TypeScript (Frontend + Chatbot UI)</span>
              <span className="tech-percent">75%</span>
            </div>
            <div className="progress-bar">
              <div className="progress-fill js-ts-fill" style={{ width: '75%', '--target-width': '75%' } as React.CSSProperties}><span>75%</span></div>
            </div>
          </div>

          <div className="progress-item">
            <div className="progress-header">
              <span className="tech-name">ROS 2 (rclpy)</span>
              <span className="tech-percent">70%</span>
            </div>
            <div className="progress-bar">
              <div className="progress-fill ros-fill" style={{ width: '70%', '--target-width': '70%' } as React.CSSProperties}><span>70%</span></div>
            </div>
          </div>

          <div className="progress-item">
            <div className="progress-header">
              <span className="tech-name">FastAPI (RAG backend)</span>
              <span className="tech-percent">80%</span>
            </div>
            <div className="progress-bar">
              <div className="progress-fill fastapi-fill" style={{ width: '80%', '--target-width': '80%' } as React.CSSProperties}><span>80%</span></div>
            </div>
          </div>

          <div className="progress-item">
            <div className="progress-header">
              <span className="tech-name">Vector DB (Qdrant)</span>
              <span className="tech-percent">65%</span>
            </div>
            <div className="progress-bar">
              <div className="progress-fill qdrant-fill" style={{ width: '65%', '--target-width': '65%' } as React.CSSProperties}><span>65%</span></div>
            </div>
          </div>

          <div className="progress-item">
            <div className="progress-header">
              <span className="tech-name">Docusaurus / MDX</span>
              <span className="tech-percent">90%</span>
            </div>
            <div className="progress-bar">
              <div className="progress-fill docusaurus-fill" style={{ width: '90%', '--target-width': '90%' } as React.CSSProperties}><span>90%</span></div>
            </div>
          </div>
        </div>
      </section>

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

      {/* WHO IS THIS BOOK FOR SECTION */}
      <section className="audience-section">
        <div className="audience-container">
          <div className="audience-image">
            <video autoPlay muted loop playsInline>
              <source src="img/new-section.mp4" type="video/mp4" />
              Your browser does not support the video tag.
            </video>
          </div>
          <div className="audience-text">
            <h2 className="audience-title">Who Is This Book For?</h2>
            <div className="audience-cards">
              <div className="audience-card" data-number="1">
                <h3>🎓 AI & Robotics Students</h3>
                <p>Perfect for those studying robotics, AI, and related fields who want practical knowledge of humanoid robot development.</p>
              </div>
              <div className="audience-card" data-number="2">
                <h3>⚙️ Developers & Engineers</h3>
                <p>Software and hardware engineers looking to build or integrate AI-powered robotic systems.</p>
              </div>
              <div className="audience-card" data-number="3">
                <h3>🔬 Researchers</h3>
                <p>Academic and industry researchers exploring the frontiers of Physical AI and humanoid robotics.</p>
              </div>
              <div className="audience-card" data-number="4">
                <h3>🚀 Tech & Innovation Enthusiasts</h3>
                <p>Technology enthusiasts interested in understanding the future of human-robot interaction and AI.</p>
              </div>

            </div>
          </div>
        </div>
      </section>

      {/* EVOLUTION TIMELINE SECTION */}
      <section className="timeline-section">
        <h2 className="timeline-title">Evolution of Humanoid Robotics</h2>
        <div className="timeline-container">
          <div className="timeline-item left">
            <div className="timeline-content">
              <h3>1970s</h3>
              <p>Early Humanoid Concepts</p>
            </div>
          </div>
          <div className="timeline-item right">
            <div className="timeline-content">
              <h3>1990s</h3>
              <p>First Walking Robots</p>
            </div>
          </div>
          <div className="timeline-item left">
            <div className="timeline-content">
              <h3>2000s</h3>
              <p>Advanced Mobility & Balance</p>
            </div>
          </div>
          <div className="timeline-item right">
            <div className="timeline-content">
              <h3>2010s</h3>
              <p>AI Integration & Interaction</p>
            </div>
          </div>
          <div className="timeline-item left">
            <div className="timeline-content">
              <h3>2020s</h3>
              <p>Commercial Applications</p>
            </div>
          </div>
          <div className="timeline-item right">
            <div className="timeline-content">
              <h3>2030s</h3>
              <p>Future Predictions</p>
            </div>
          </div>
        </div>
      </section>

      {/* CHAT BOX FLOATING */}
      <BookChat />
    </Layout>
  );
}