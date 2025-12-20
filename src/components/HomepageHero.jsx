import React from 'react';
import Link from '@docusaurus/Link';
import styles from '../css/homepage.module.css';

export default function HomepageHero() {
  return (
    <div className={styles.homepageWrapper}>
      {/* Hero Section */}
      <div className={styles.heroSection}>
        {/* Animated Background */}
        <div className={styles.backgroundOverlay}>
          <div className={styles.gridLines}></div>
          <div className={styles.glowOrb1}></div>
          <div className={styles.glowOrb2}></div>
        </div>

        {/* Main Content */}
        <div className={styles.heroContent}>
          {/* Robot Image/Visual */}
          <div className={styles.robotVisual}>
            <div className={styles.robotContainer}>
              <div className={styles.robotGlow}></div>
              <span className={styles.robotEmoji}>🤖</span>
            </div>
          </div>

          {/* Text Content */}
          <div className={styles.textContent}>
            <div className={styles.badge}>
              <span>🎓</span> University Capstone Course
            </div>

            <h1 className={styles.mainTitle}>
              Physical AI &<br/>
              <span className={styles.highlight}>Humanoid Robotics</span>
            </h1>

            <p className={styles.subtitle}>
              Zero to Walking Robot in 13 Weeks
            </p>

            <p className={styles.description}>
              A production-grade educational resource for building AI-controlled
              humanoid robots with ROS2, NVIDIA Isaac Sim, and LLM integration.
            </p>

            <p className={styles.author}>
              by <span className={styles.authorName}>Syeda Farheen Zehra</span>
            </p>

            <div className={styles.buttonGroup}>
              <Link
                className={styles.primaryButton}
                to="/docs/">
                📖 Start Reading
              </Link>
              <Link
                className={styles.secondaryButton}
                to="/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve">
                🚀 Jump to Module 1
              </Link>
            </div>

            <div className={styles.features}>
              <div className={styles.featureItem}>
                <span>✅</span> Zero Broken Code
              </div>
              <div className={styles.featureItem}>
                <span>✅</span> Production-Grade
              </div>
              <div className={styles.featureItem}>
                <span>✅</span> Copy-Paste Ready
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Core Research Areas Section */}
      <section className={styles.researchSection}>
        <div className={styles.sectionContainer}>
          <h2 className={styles.sectionTitle}>Core Research Areas</h2>
          <p className={styles.sectionSubtitle}>
            Master the essential technologies powering modern humanoid robotics
          </p>

          <div className={styles.cardsGrid}>
            {/* ROS 2 Card */}
            <div className={styles.researchCard}>
              <div className={styles.cardIcon}>🔧</div>
              <h3 className={styles.cardTitle}>ROS 2 Ecosystem</h3>
              <p className={styles.cardDescription}>
                Advanced middleware for complex humanoid robot control, including nodes,
                topics, services, and URDF for robot description.
              </p>
              <Link to="/docs/module-01-foundations/chapter-01-ros2-nervous-system/page-01-what-problem-does-ros2-solve" className={styles.cardLink}>
                Explore ROS 2 →
              </Link>
            </div>

            {/* Digital Twin Card */}
            <div className={styles.researchCard}>
              <div className={styles.cardIcon}>🌐</div>
              <h3 className={styles.cardTitle}>Digital Twin Simulation</h3>
              <p className={styles.cardDescription}>
                Physics-based simulation environments using Gazebo and Unity for testing
                humanoid locomotion and behaviors in virtual worlds.
              </p>
              <Link to="/docs/module-02-digital-twin/chapter-04-gazebo-physics-simulation/page-10-why-simulation-matters" className={styles.cardLink}>
                Explore Simulation →
              </Link>
            </div>

            {/* NVIDIA Isaac Card */}
            <div className={styles.researchCard}>
              <div className={styles.cardIcon}>🚀</div>
              <h3 className={styles.cardTitle}>NVIDIA Isaac AI</h3>
              <p className={styles.cardDescription}>
                Hardware-accelerated AI for perception, navigation, and manipulation
                using Isaac Sim and Isaac ROS packages.
              </p>
              <Link to="/docs/module-03-ai-robot-brain/chapter-07-isaac-sim-sdk/page-20-setting-up-isaac-sdk" className={styles.cardLink}>
                Explore Isaac →
              </Link>
            </div>
          </div>
        </div>
      </section>

      {/* Interactive Assistant Section */}
      <section className={styles.assistantSection}>
        <div className={styles.sectionContainer}>
          <div className={styles.assistantContent}>
            <div className={styles.assistantText}>
              <h2 className={styles.sectionTitle}>Interactive Robotics Assistant</h2>
              <p className={styles.assistantDescription}>
                Get personalized help with our RAG-powered AI assistant. Ask questions about
                robotics, ROS2, simulation, and more - all grounded in our comprehensive textbook content.
              </p>
              <ul className={styles.assistantFeatures}>
                <li>📚 Answers grounded in textbook content</li>
                <li>🔍 Smart context retrieval</li>
                <li>💡 Personalized to your learning level</li>
                <li>🌐 Available 24/7</li>
              </ul>
              <div className={styles.assistantCta}>
                <Link to="/auth/signup" className={styles.primaryButton}>
                  Sign Up for Full Access
                </Link>
                <Link to="/auth/signin" className={styles.secondaryButton}>
                  Sign In
                </Link>
              </div>
            </div>
            <div className={styles.assistantPreview}>
              <div className={styles.chatPreview}>
                <div className={styles.chatHeader}>
                  <span className={styles.chatDot}></span>
                  <span className={styles.chatTitle}>Robotics Assistant</span>
                </div>
                <div className={styles.chatMessages}>
                  <div className={styles.chatBubbleBot}>
                    Hello! I'm your RAG Chatbot for robotics. Ask me anything about ROS2,
                    simulation, or humanoid robots!
                  </div>
                  <div className={styles.chatBubbleUser}>
                    What is ROS2?
                  </div>
                  <div className={styles.chatBubbleBot}>
                    ROS2 (Robot Operating System 2) is middleware that provides tools and
                    libraries for building robot applications...
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className={styles.ctaSection}>
        <div className={styles.sectionContainer}>
          <h2 className={styles.ctaTitle}>Ready to Advance Your Research?</h2>
          <p className={styles.ctaDescription}>
            Join our community of researchers and developers pushing the boundaries of humanoid robotics
          </p>
          <div className={styles.ctaButtons}>
            <Link to="/docs/" className={styles.primaryButton}>
              Get Started
            </Link>
            <Link to="/auth/signup" className={styles.secondaryButton}>
              Sign Up
            </Link>
          </div>
        </div>
      </section>
    </div>
  );
}