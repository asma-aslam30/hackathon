import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import AnimatedBackground from '@site/src/components/AnimatedBackground';
import StatsCounter from '@site/src/components/StatsCounter';
import Carousel from '@site/src/components/Carousel';
import InteractiveCard from '@site/src/components/InteractiveCard';
import ImageShowcase from '@site/src/components/ImageShowcase';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// SVG Icons
const RocketIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M4.5 16.5c-1.5 1.25-2 5-2 5s3.75-.5 5-2c.625-.625 1-1.5 1-2.5 0-1-.375-1.875-1-2.5-.625-.625-1.5-1-2.5-1-.625 0-1.25.125-1.5.5z" />
    <path d="M12 15l-3-3a22 22 0 0 1 2-3.95A12.88 12.88 0 0 1 22 2c0 2.72-.78 7.5-6 11a22.35 22.35 0 0 1-4 2z" />
    <path d="M9 12H4s.55-3.03 2-4c1.62-1.08 5 0 5 0" />
    <path d="M12 15v5s3.03-.55 4-2c1.08-1.62 0-5 0-5" />
  </svg>
);

const SparklesIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M12 3v18m9-9H3m15.364 6.364L5.636 5.636m12.728 0L5.636 18.364" />
  </svg>
);

const DevicesIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <rect x="2" y="3" width="20" height="14" rx="2" />
    <path d="M8 21h8m-4-4v4" />
  </svg>
);

const CodeIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <polyline points="16 18 22 12 16 6" />
    <polyline points="8 6 2 12 8 18" />
  </svg>
);

const AccessibleIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <circle cx="12" cy="12" r="10" />
    <path d="M12 16v-4m0-4h.01" />
  </svg>
);

const HeartIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M20.84 4.61a5.5 5.5 0 0 0-7.78 0L12 5.67l-1.06-1.06a5.5 5.5 0 0 0-7.78 7.78l1.06 1.06L12 21.23l7.78-7.78 1.06-1.06a5.5 5.5 0 0 0 0-7.78z" />
  </svg>
);

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <AnimatedBackground variant="mesh" intensity="medium" />

      <div className={styles.heroContent}>
        <div className={clsx(styles.heroGlass, 'animate-fade-in-up')}>
          <div className={styles.heroBadge}>
            <SparklesIcon />
            <span>Next-Gen Documentation</span>
          </div>

          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>

          <div className={styles.heroButtons}>
            <Link
              className={clsx('button button--lg', styles.primaryButton)}
              to="/docs/intro">
              <RocketIcon />
              <span>Get Started</span>
              <span className={styles.buttonGlow} aria-hidden="true" />
            </Link>
            <Link
              className={clsx('button button--lg', styles.secondaryButton)}
              to="/docs/intro">
              <CodeIcon />
              <span>View Docs</span>
            </Link>
          </div>

          <div className={styles.heroFeatures}>
            <div className={styles.heroFeature}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 2l3.09 6.26L22 9.27l-5 4.87 1.18 6.88L12 17.77l-6.18 3.25L7 14.14 2 9.27l6.91-1.01L12 2z" />
              </svg>
              <span>Open Source</span>
            </div>
            <div className={styles.heroFeature}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M13 10V3L4 14h7v7l9-11h-7z" />
              </svg>
              <span>Lightning Fast</span>
            </div>
            <div className={styles.heroFeature}>
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z" />
              </svg>
              <span>Production Ready</span>
            </div>
          </div>
        </div>

        <div className={styles.scrollIndicator}>
          <div className={styles.scrollArrow} />
        </div>
      </div>
    </header>
  );
}

function StatsSection() {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          <StatsCounter
            end={10000}
            suffix="+"
            label="Active Users"
            icon="👥"
            accentColor="cyan"
          />
          <StatsCounter
            end={500}
            suffix="+"
            label="Projects Built"
            icon="🚀"
            accentColor="purple"
          />
          <StatsCounter
            end={99}
            suffix="%"
            label="Satisfaction Rate"
            icon="⭐"
            accentColor="pink"
          />
          <StatsCounter
            end={24}
            suffix="/7"
            label="Support Available"
            icon="💬"
            accentColor="green"
          />
        </div>
      </div>
    </section>
  );
}

function TestimonialsSection() {
  const testimonials = [
    {
      quote: "This is the most beautiful documentation site I've ever seen. The interactive elements make learning a joy!",
      author: "Sarah Chen",
      role: "Frontend Developer",
      avatar: "👩‍💻",
    },
    {
      quote: "The cyber-minimalist design is stunning. It's both functional and visually impressive.",
      author: "Alex Kumar",
      role: "UI/UX Designer",
      avatar: "🎨",
    },
    {
      quote: "Amazing performance and accessibility. The animations are smooth and the contrast is perfect.",
      author: "Jordan Smith",
      role: "Tech Lead",
      avatar: "👨‍💼",
    },
  ];

  return (
    <section className={styles.testimonialsSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          What People Say
        </Heading>
        <p className={styles.sectionSubtitle}>
          Loved by developers and designers worldwide
        </p>

        <Carousel autoPlay interval={6000} className={styles.testimonialCarousel}>
          {testimonials.map((testimonial, idx) => (
            <InteractiveCard
              key={idx}
              variant="glass"
              accentColor={idx === 0 ? 'cyan' : idx === 1 ? 'purple' : 'pink'}
              className={styles.testimonialCard}
            >
              <div className={styles.testimonialAvatar}>{testimonial.avatar}</div>
              <p className={styles.testimonialQuote}>"{testimonial.quote}"</p>
              <div className={styles.testimonialAuthor}>
                <strong>{testimonial.author}</strong>
                <span>{testimonial.role}</span>
              </div>
            </InteractiveCard>
          ))}
        </Carousel>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Modern documentation with interactive UI and cyber-minimalist design">
      <HomepageHeader />
      <main className={styles.mainContent}>
        <StatsSection />
        <HomepageFeatures />
        <TestimonialsSection />
      </main>
    </Layout>
  );
}
