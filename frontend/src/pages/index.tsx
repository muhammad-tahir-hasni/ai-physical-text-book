import type {ReactNode} from 'react';
import {useState} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import {Redirect} from '@docusaurus/router';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import {useAuth} from '@site/src/context/AuthContext';
import AuthModal from '@site/src/components/AuthModal';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const {isAuthenticated} = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const docsUrl = useBaseUrl('/docs/intro');

  const handleGetStarted = (e: React.MouseEvent) => {
    if (!isAuthenticated) {
      e.preventDefault();
      setShowAuthModal(true);
    }
  };

  return (
    <>
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            {isAuthenticated ? (
              <Link
                className="button button--secondary button--lg"
                to={docsUrl}>
                Start Learning
              </Link>
            ) : (
              <button
                className="button button--secondary button--lg"
                onClick={handleGetStarted}>
                Get Started
              </button>
            )}
          </div>
        </div>
      </header>

      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        initialMode="signup"
      />
    </>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const {isAuthenticated, isLoading} = useAuth();
  const docsUrl = useBaseUrl('/docs/intro');

  // Show loading state while checking authentication
  if (isLoading) {
    return (
      <Layout title="Loading...">
        <div style={{ textAlign: 'center', padding: '4rem' }}>
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  // Redirect authenticated users to course intro
  if (isAuthenticated) {
    return <Redirect to={docsUrl} />;
  }

  // Show landing page for non-authenticated users
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Interactive Textbook for Learning ROS 2, Simulation & Vision-Language-Action Systems">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
