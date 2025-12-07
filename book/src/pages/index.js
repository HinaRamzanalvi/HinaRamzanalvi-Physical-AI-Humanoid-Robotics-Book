import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

// Custom Hero Section Component
function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--clean', styles.heroBanner)}>
      <div className="container text--center">
        <Heading as="h1" className={clsx(styles.heroTitle, styles.fadeIn)} style={{ animationDelay: '0.5s' }}>
          {siteConfig.title}
        </Heading>
        <p className={clsx(styles.heroSubtitle, styles.fadeIn)} style={{ animationDelay: '1.0s' }}>
          A comprehensive course on Physical AI and Humanoid Robotics
        </p>
        <div className={clsx(styles.buttons, styles.fadeIn)} style={{ animationDelay: '1.5s' }}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Learning
          </Link>
          <Link
            className="button button--secondary button--lg"
            to="/docs/module1_ros2/">
            Explore Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

const LatestModules = [
  {
    title: 'Module 1: Robotic Nervous System (ROS 2)',
    description: 'Learn the fundamentals of ROS 2 for robot communication and control.',
    link: '/docs/module1_ros2/',
  },
  {
    title: 'Module 2: Digital Twin (Gazebo & Unity)',
    description: 'Build and simulate realistic robot environments using Gazebo and Unity.',
    link: '/docs/module2_digital_twin/',
  },
  {
    title: 'Module 3: AI-Robot Brain (NVIDIA Isaac)',
    description: 'Explore AI perception, planning, and learning with NVIDIA Isaac Sim and ROS.',
    link: '/docs/module3_ai_robot_brain/',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Empower robots to understand and act on human voice and language commands.',
    link: '/docs/module4_vla/',
  },
];

function LatestModulesSection() {
  return (
    <section className={clsx('margin-bottom--xl', styles.latestModulesSection)}>
      <div className="container text--center">
        <Heading as="h2" className={styles.sectionTitle}>
          Latest Modules & Chapters
        </Heading>
        <p className={clsx('hero__subtitle', 'margin-bottom--lg', styles.heroSubtitle)}>Dive into the newest advancements in Physical AI and Humanoid Robotics.</p>
        <div className={styles.moduleCardsContainer}>
          {LatestModules.map((module, idx) => (
            <Link to={module.link} key={idx} className={styles.moduleCard}>
              <h3 className={styles.moduleCardTitle}>{module.title}</h3>
              <p className={styles.moduleCardDescription}>{module.description}</p>
              <span className={styles.moduleCardLink}>Explore Module →</span>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive course on Physical AI and Humanoid Robotics, covering ROS 2, Digital Twins, NVIDIA Isaac, and VLA models.">
      <HomepageHero />
      <main>
        <HomepageFeatures />
        <LatestModulesSection />
      </main>
    </Layout>
  );
}
