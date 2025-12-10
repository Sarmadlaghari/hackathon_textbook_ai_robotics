import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Complete Curriculum',
    description: (
      <>
        4 Modules covering 13 weeks of comprehensive content in Physical AI and Humanoid Robotics.
      </>
    ),
  },
  {
    title: 'Industry-Standard Tools',
    description: (
      <>
        Learn with ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems used in real robotics applications.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    description: (
      <>
        Practical exercises and examples that bridge theory with implementation in robotics systems.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Technologies',
    description: (
      <>
        Explore the latest advances in AI-Physical interaction, simulation, and autonomous systems.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
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