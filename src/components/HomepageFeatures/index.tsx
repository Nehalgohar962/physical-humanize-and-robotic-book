import React, { useState } from 'react';
import styles from './styles.module.css';

type ModuleItem = {
  title: string;
  description: React.ReactNode;
};

const ModuleList: ModuleItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: (
      <>
        <h4>Introduction to Physical AI & Humanoid Robotics</h4>
        <p>Target Audience: Students and developers with a basic understanding of computer science, Python, and command-line interfaces. No prior ROS 2 experience is required, but familiarity with robotics concepts is beneficial.</p>

        <h5>Learning Goals:</h5>
        <ul>
          <li>Understand the fundamental concepts of ROS 2: nodes, topics, services, actions, and parameters.</li>
          <li>Learn how to create and run ROS 2 nodes in Python.</li>
          <li>Master the pub/sub communication pattern for data streaming.</li>
          <li>Implement request/response communication using ROS 2 services.</li>
          <li>Gain hands-on experience with ROS 2 command-line tools for introspection and debugging.</li>
        </ul>

        <h5>1.1 Introduction: What is a Robot's Nervous System?</h5>
        <p>Just as a biological organism relies on a nervous system to perceive, process, and act, a robot relies on a sophisticated software framework. This framework acts as the robot's digital nervous system, managing communication between its various components—sensors (eyes, ears), actuators (motors, grippers), and the "brain" (computational units).</p>

        <h5>1.2 The Core of ROS 2: Nodes</h5>
        <p>Everything in ROS 2 is built around the concept of a Node. A node is the smallest unit of computation. Think of it as a single, dedicated process responsible for one specific task.</p>
        <ul>
          <li>A node for reading data from a camera.</li>
          <li>A node for controlling the wheel motors.</li>
          <li>A node for processing sensor data to detect obstacles.</li>
          <li>A node for planning a path from point A to point B.</li>
        </ul>
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: <p>Content for Module 2 goes here...</p>,
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: <p>Content for Module 3 goes here...</p>,
  },
];

type ModuleProps = {
  title: string;
  description: React.ReactNode;
  isOpen: boolean;
  onClick: () => void;
};

const Module: React.FC<ModuleProps> = ({ title, description, isOpen, onClick }) => {
  return (
    <div className={styles.moduleItem}>
      <button
        onClick={onClick}
        className={styles.moduleButton}
      >
        {title}
      </button>
      {isOpen && (
        <div className={styles.moduleContent}>
          {description}
        </div>
      )}
    </div>
  );
};

const HomepageFeatures: React.FC = () => {
  const [openModule, setOpenModule] = useState<number | null>(null);

  return (
    <section className={styles.featuresSection}>
      <div className={styles.featuresContainer}>
        {ModuleList.map((module, idx) => (
          <Module
            key={idx}
            title={module.title}
            description={module.description}
            isOpen={openModule === idx}
            onClick={() => setOpenModule(openModule === idx ? null : idx)}
          />
        ))}
      </div>
    </section>
  );
};

export default HomepageFeatures;
