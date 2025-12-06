import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro"
          >
            Read the Book - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive guide to integrating advanced robotics into educational contexts"
    >
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Educational Robotics</h3>
                <p>
                  Learn how humanoid robots can transform educational
                  experiences for students of all ages.
                </p>
              </div>
              <div className="col col--4">
                <h3>Technical Implementation</h3>
                <p>
                  Understand the technical concepts and practical approaches to
                  implementing robotic systems in education.
                </p>
              </div>
              <div className="col col--4">
                <h3>Ethical Considerations</h3>
                <p>
                  Explore the ethical dilemmas and responsible practices in
                  educational robotics.
                </p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
