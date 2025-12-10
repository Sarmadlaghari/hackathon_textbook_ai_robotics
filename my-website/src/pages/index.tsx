import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 13 Week Curriculum
          </Link>
          <Link
            className="button button--primary button--lg margin-left--sm"
            to="https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModuleCard({ title, weeks, description }: { title: string; weeks: string; description: string }) {
  // Map the displayed title to the actual directory name
  const getModulePath = (title: string): string => {
    if (title.includes('Module 1')) return 'module-1-robotic-nervous-system';
    if (title.includes('Module 2')) return 'module-2-digital-twin';
    if (title.includes('Module 3')) return 'module-3-ai-robot-brain';
    if (title.includes('Module 4')) return 'module-4-vision-language-action';
    return 'intro'; // fallback
  };

  return (
    <div className="col col--3 margin-bottom--lg">
      <div className="card">
        <div className="card__header">
          <h3>{title}</h3>
          <small>{weeks}</small>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--secondary button--block" to={`/docs/${getModulePath(title)}`}>
            Explore Module
          </Link>
        </div>
      </div>
    </div>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modules}>
      <div className="container padding-vert--lg">
        <Heading as="h2" className="text--center margin-bottom--lg">
          Curriculum Overview
        </Heading>
        <div className="row">
          <ModuleCard
            title="Module 1 — The Robotic Nervous System (ROS 2)"
            weeks="Weeks 1-3"
            description="Introduction to Physical AI and Sensors (LIDAR, IMUs), ROS 2 Fundamentals (Nodes, Topics, Services, Packages), Python Agent Integration with ROS Controllers + URDF Modeling"
          />
          <ModuleCard
            title="Module 2 — The Digital Twin (Gazebo & Unity)"
            weeks="Weeks 4-5"
            description="Physics Simulation in Gazebo (Gravity, Collisions), High-Fidelity Rendering in Unity + Sensor Simulation"
          />
          <ModuleCard
            title="Module 3 — The AI-Robot Brain (NVIDIA Isaac™)"
            weeks="Weeks 6-8"
            description="NVIDIA Isaac Sim (Photorealistic Simulation), Isaac ROS (Hardware-Accelerated VSLAM + Nav2), Isaac Sim for Reinforcement Learning"
          />
          <ModuleCard
            title="Module 4 — Vision-Language-Action (VLA)"
            weeks="Weeks 9-13"
            description="Voice-to-Action with OpenAI Whisper, Cognitive Planning (LLMs Translating Natural Language to ROS 2 Actions), Capstone (Autonomous Humanoid Deployment & Testing)"
          />
        </div>
      </div>
    </section>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.features}>
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>Comprehensive Coverage</h3>
              <p>Complete 13-week curriculum covering the latest in Physical AI and Humanoid Robotics</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>Hands-on Learning</h3>
              <p>Practical examples and exercises using industry-standard tools and frameworks</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>Industry Ready</h3>
              <p>Learn with the same technologies used in cutting-edge robotics research and development</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="AI-Native Textbook for Physical AI & Humanoid Robotics covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        <FeaturesSection />
        <ModulesSection />
      </main>
    </Layout>
  );
}