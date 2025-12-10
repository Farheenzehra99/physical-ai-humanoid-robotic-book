import React from 'react';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from '../css/homepage.module.css';

export default function HomepageHero() {
  return (
    <div className={styles.heroSection}>
      <div className={styles.bookCoverContainer}>
        <div className={styles.bookCoverPlaceholder}>
          <div className={styles.bookCoverImage}>📚</div>
        </div>
      </div>

      <div className={styles.contentContainer}>
        <h1 className={styles.title}>
          Physical AI & Humanoid Robotics
        </h1>
        <p className={styles.subtitle}>
          A Practical Guide to Embodied Intelligence, Robotics Simulation, and LLM-driven Control
        </p>
        <p className={styles.author}>by Syeda Farheen Zehra</p>

        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="https://github.com/your-org/physical-ai-humanoid-robotics">
            Open Source
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Reading →
          </Link>
        </div>

        <div className={styles.additionalButtons}>
          <Link
            className="button button--outline button--md"
            to="/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve">
            Co-Learning with AI
          </Link>
          <Link
            className="button button--outline button--md"
            to="/docs/intro">
            Spec-Driven Development
          </Link>
        </div>
      </div>
    </div>
  );
}