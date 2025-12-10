import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroTextSection}>
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Get Started - Read the Book
              </Link>
              <Link
                className="button button--primary button--lg"
                to="/docs/setup/ubuntu-preparation">
                Start Setup Guide
              </Link>
            </div>
            {/* Technology stack badges */}
            <div className={styles.techStack}>
              <span className="tech-badge">ROS 2</span>
              <span className="tech-badge">NVIDIA Isaac</span>
              <span className="tech-badge">Gazebo</span>
              <span className="tech-badge">Unity</span>
              <span className="tech-badge">Python</span>
              <span className="tech-badge">C++</span>
            </div>
          </div>
          <div className={styles.heroImageSection}>
            <img
              src={require('../../static/img/robot-hero.png').default}
              alt="Futuristic humanoid robot for Physical AI & Humanoid Robotics educational book"
              className={styles.heroRobotImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A Comprehensive Educational Book on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        {/* Key Learning Outcomes */}
        <section className={styles.outcomes}>
          <div className="container">
            <h2 className={styles.sectionTitle}>What You'll Achieve</h2>
            <div className="row">
              <div className="col col--3">
                <div className="card">
                  <div className="card__body text--center">
                    <h3>✅ 90% Navigation Success</h3>
                    <p>Build autonomous navigation systems with high success rates</p>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card">
                  <div className="card__body text--center">
                    <h3>🤖 Humanoid Control</h3>
                    <p>Implement advanced humanoid robot control systems</p>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card">
                  <div className="card__body text--center">
                    <h3>⚡ 500ms Response</h3>
                    <p>Develop real-time perception and response systems</p>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card">
                  <div className="card__body text--center">
                    <h3>🎯 Sim-to-Real Transfer</h3>
                    <p>Deploy models from simulation to real hardware</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Enhanced Feature Cards */}
        <section className={clsx(styles.features, styles.featuresGrid)}>
          <div className="container">
            <h2 className={styles.sectionTitle}>Learning Modules</h2>
            <div className="row">
              <div className="col col--3">
                <div className="card">
                  <div className="card__header text--center">
                    <h3>The Robotic Nervous System (ROS 2)</h3>
                  </div>
                  <div className="card__body">
                    <p>Learn the fundamentals of Physical AI, ROS 2, and environment setup.</p>
                  </div>
                  <div className="card__footer text--center">
                    <Link className="button button--primary button--block" to="/docs/ros2-fundamentals/nodes-topics">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card">
                  <div className="card__header text--center">
                    <h3>The Digital Twin (Gazebo & Unity)</h3>
                  </div>
                  <div className="card__body">
                    <p>Build physics-accurate robot simulations with Gazebo and advanced modeling.</p>
                  </div>
                  <div className="card__footer text--center">
                    <Link className="button button--primary button--block" to="/docs/simulation/gazebo-installation">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card">
                  <div className="card__header text--center">
                    <h3>The AI-Robot Brain (NVIDIA Isaac™)</h3>
                  </div>
                  <div className="card__body">
                    <p>Implement perception, navigation, and VSLAM systems for autonomous robots.</p>
                  </div>
                  <div className="card__footer text--center">
                    <Link className="button button--primary button--block" to="/docs/isaac-platform/isaac-ros-installation">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>
              <div className="col col--3">
                <div className="card">
                  <div className="card__header text--center">
                    <h3>Vision-Language-Action (VLA)</h3>
                  </div>
                  <div className="card__body">
                    <p>Integrate LLMs, voice control, and advanced AI for autonomous behavior.</p>
                  </div>
                  <div className="card__footer text--center">
                    <Link className="button button--primary button--block" to="/docs/vla/integration">
                      Start Learning
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className={styles.description}>
          <div className="container">
            <div className="row">
              <div className="col col--12">
                <h2 className={styles.sectionTitle}>About This Book</h2>
                <p>
                  This comprehensive educational book is designed for advanced undergraduate and graduate students
                  with a background in Python and machine learning fundamentals. The curriculum takes you through
                  a complete 13-week program that progresses from foundational concepts to advanced implementations.
                </p>
                <p>
                  By the end of this book, you will be able to set up complete ROS 2 development environments,
                  create physics-accurate robot simulations, implement VSLAM and navigation pipelines using Isaac ROS,
                  integrate LLMs for voice-controlled robot actions, and deploy AI models from simulation to real hardware.
                </p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}