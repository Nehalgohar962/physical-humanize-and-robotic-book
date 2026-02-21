// import React from 'react';
// import Layout from '@theme/Layout';
// import Link from '@docusaurus/Link';
// import '../css/home.css';

// const modules = [
//   {
//     title: 'Module 1: The Robotic Nervous System (ROS 2)',
//     desc: 'Understand ROS2, nodes, topics, packages, and launching robotic systems.',
//     path: '/docs/module1',
//   },
//   {
//     title: 'Module 2: The Digital Twin (Gazebo & Unity)',
//     desc: 'Learn simulation, digital twin creation, and robot environment modeling.',
//     path: '/docs/module2',
//   },
//   {
//     title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
//     desc: 'Study AI-powered robotics, sensor fusion models, and robot intelligence.',
//     path: '/docs/module3',
//   },
//   {
//     title: 'Module 4: Vision-Language-Action (VLA)',
//     desc: 'Learn multimodal AI, VLMs, VLA systems, and robot learning.',
//     path: '/docs/module4',
//   },
//   {
//     title: 'Module 5: Advanced AI & Motion Control',
//     desc: 'Deep dive into locomotion, manipulation, reinforcement learning, control.',
//     path: '/docs/module5',
//   },
//   {
//     title: 'Introduction to Physical AI & Robotics',
//     desc: 'Foundation overview of Physical AI and Human-Robot synergy.',
//     path: '/docs/intro',
//   },
// ];

// export default function Modules() {
//   return (
//     <Layout title="Modules" description="All Chapters of the Book">
//       <div className="modules-grid">
//         {modules.map((mod, idx) => (
//           <div className="module-card" key={idx}>
//             <h3>{mod.title}</h3>
//             <p>{mod.desc}</p>
//             <Link className="read-btn" to={mod.path}>
//               Read Module ➜
//             </Link>
//           </div>
//         ))}
//       </div>
//     </Layout>
//   );
// }





import React from 'react';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import '../css/custom.css';

export default function Modules() {
  return (
    <Layout title="Modules - Physical AI & Humanoid Robotics">
      <section className="modules-heading">
        <h2>Explore All Modules</h2>
        <p>Step into the world of Physical AI and Humanoid Robotics through these structured modules.</p>
      </section>

      <section className="modules-grid">
        <div className="module-card">
          <h3>Module 1: The Robotic Nervous System (ROS 2)</h3>
          <p>Target Audience: Students and developers with a basic understanding of computer science, Python, and command-line interfaces. No prior ROS 2 experience is required, but familiarity with robotics concepts is beneficial.</p>
          <Link className="read-btn" to="/docs/module1">Read ➜</Link>
        </div>
        <div className="module-card">
          <h3>Module 2: The Digital Twin (Gazebo & Unity)</h3>
          <p>In robotics, a Digital Twin is a virtual replica of a physical robot. This is not just a 3D model; it's a dynamic simulation that includes the robot's physical properties (mass, inertia), sensors, actuators, and the environment it operates in.Why is this so important?</p>
          <Link className="read-btn" to="/docs/module2">Read ➜</Link>
        </div>
        <div className="module-card">
          <h3>Module 3: The AI-Robot Brain (NVIDIA Isaac™)</h3>
          <p>In Module 2, you learned how to simulate a robot in Gazebo. While Gazebo is an excellent tool for many robotics tasks, the world of AI-driven robotics often demands a higher level of realism, especially for training perception and interaction algorithms.</p>
          <Link className="read-btn" to="/docs/module3">Read ➜</Link>
        </div>
        <div className="module-card">
          <h3>Module 4: Vision-Language-Action (VLA)</h3>
          <p>We have reached the final and most exciting module. So far, we have built the robot's nervous system (ROS 2), given it a virtual body (Gazebo/Isaac), and provided it with an AI-ready brain (Isaac Sim).</p>
          <Link className="read-btn" to="/docs/module4">Read ➜</Link>
        </div>
        <div className="module-card">
          <h3>Module 5: Advanced AI & Motion Control</h3>
          <p>In previous modules, we established the robot's nervous system with ROS 2 and its digital twin in high-fidelity simulators. Now, we delve into the "brain" itself: how advanced AI techniques enable humanoid robots to move intelligently, adapt to their environment, </p>
          <Link className="read-btn" to="/docs/module5">Read ➜</Link>
        </div>
        <div className="module-card">
          <h3>Module 6: Designing Humanoid Robots</h3>
          <p>Designing a humanoid robot is a complex endeavor that blends art and science, requiring expertise in mechanical engineering, electronics, computer science, and artificial intelligence. Unlike industrial robots with fixed tasks, humanoids are designed to</p>
          <Link className="read-btn" to="/docs/module6">Read ➜</Link>
        </div>
      </section>
    </Layout>
  );
}
