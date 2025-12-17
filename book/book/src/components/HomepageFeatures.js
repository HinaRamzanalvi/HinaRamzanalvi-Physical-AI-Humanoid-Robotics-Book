import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI Fundamentals',
    // Svg: require('@site/static/img/undraw_robotics.svg').default,
    description: (
      <>
        Understand the core principles of embodied intelligence and how AI interacts with the physical world.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    // Svg: require('@site/static/img/undraw_robot_arm.svg').default,
    description: (
      <>
        Dive into the mechanics and control of humanoid robots, from kinematics to advanced motion planning.
      </>
    ),
  },
  {
    title: 'Simulation & Digital Twins',
    // Svg: require('@site/static/img/undraw_city_driver.svg').default,
    description: (
      <>
        Learn to build and interact with digital twins using Gazebo, Unity, and NVIDIA Isaac Sim.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    // Svg: require('@site/static/img/undraw_artificial_intelligence.svg').default,
    description: (
      <>
        Integrate cutting-edge AI for voice control, natural language understanding, and autonomous task completion.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--3', styles.featureCard)}>
      {/* <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div> */}
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
