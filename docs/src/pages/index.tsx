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

const CodeIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <polyline points="16 18 22 12 16 6" />
    <polyline points="8 6 2 12 8 18" />
  </svg>
);

const BrainIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M12 4.5a2.5 2.5 0 0 0-4.96-.46 2.5 2.5 0 0 0-2.48 3 2.5 2.5 0 0 0-.06 4.5 2.5 2.5 0 0 0 3.06 2.96A2.5 2.5 0 0 0 10.5 19.5a2.5 2.5 0 0 0 4.96-.46 2.5 2.5 0 0 0 2.48-3 2.5 2.5 0 0 0 .06-4.5 2.5 2.5 0 0 0-3.06-2.96A2.5 2.5 0 0 0 12 4.5Z" />
  </svg>
);

const RobotIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <rect x="3" y="11" width="18" height="10" rx="2" />
    <circle cx="9" cy="6" r="2" />
    <circle cx="15" cy="6" r="2" />
    <path d="M10 16h.01M14 16h.01" />
  </svg>
);

const GazeboIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M12 3l2 5 5 .5-4 3.5 1 5-4.5-2.5L12 20l-1-5L6.5 12.5 12 10l1-5z" />
  </svg>
);

const IsaacIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M20 10c0 4.4-3.6 8-8 8s-8-3.6-8-8 3.6-8 8-8 8 3.6 8 8z" />
    <circle cx="12" cy="10" r="3" />
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
            <span>Physical AI & Robotics Platform</span>
          </div>

          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>

          <div className={styles.heroButtons}>
            <Link
              className={clsx('button button--lg', styles.primaryButton)}
              to="/docs/physical-ai-robotics/introduction">
              <RocketIcon />
              <span>Get Started</span>
            </Link>
            <Link
              className={clsx('button button--lg', styles.secondaryButton)}
              to="/chatbot">
              <SparklesIcon />
              <span>AI Chatbot</span>
            </Link>
            <Link
              className={clsx('button button--lg', styles.secondaryButton)}
              to="/docs/physical-ai-robotics/introduction">
              <CodeIcon />
              <span>View Documentation</span>
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
              <span>AI-Powered</span>
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

function ModulesSection() {
  const modules = [
    {
      title: "ROS 2 - Robotic Nervous System",
      icon: <BrainIcon />,
      description: "The de facto standard for robotic middleware with advanced communication patterns",
      color: "cyan",
      to: "/docs/physical-ai-robotics/module-1-ros2/overview"
    },
    {
      title: "Digital Twin - Gazebo & Unity",
      icon: <RobotIcon />,
      description: "High-fidelity simulation environments for development and testing",
      color: "purple",
      to: "/docs/physical-ai-robotics/module-2-digital-twin/overview"
    },
    {
      title: "AI-Robot Brain - Isaac Sim",
      icon: <IsaacIcon />,
      description: "NVIDIA Isaac Sim for advanced robotics and AI development",
      color: "pink",
      to: "/docs/physical-ai-robotics/module-3-isaac-brain/overview"
    },
    {
      title: "Vision-Language-Action",
      icon: <GazeboIcon />,
      description: "Intelligent systems combining perception, reasoning, and action",
      color: "green",
      to: "/docs/physical-ai-robotics/module-4-vla/overview"
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Core Modules
        </Heading>
        <p className={styles.sectionSubtitle}>
          Comprehensive modules covering all aspects of Physical AI & Robotics
        </p>

        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <Link to={module.to} key={index} className={styles.moduleCard}>
              <InteractiveCard
                variant="glass"
                accentColor={module.color}
                className={styles.moduleCardInner}
              >
                <div className={clsx(styles.moduleIcon, styles[`icon-${module.color}`])}>
                  {module.icon}
                </div>
                <h3 className={styles.moduleTitle}>{module.title}</h3>
                <p className={styles.moduleDescription}>{module.description}</p>
              </InteractiveCard>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function StatsSection() {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          <StatsCounter
            end={15000}
            suffix="+"
            label="Lines of Code"
            icon="💻"
            accentColor="cyan"
          />
          <StatsCounter
            end={4}
            suffix="+"
            label="Core Modules"
            icon="📚"
            accentColor="purple"
          />
          <StatsCounter
            end={100}
            suffix="+"
            label="Code Examples"
            icon="🧪"
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
      quote: "The Physical AI & Robotics platform is a game-changer for robotic development. The combination of ROS 2, simulation environments, and AI capabilities is unparalleled.",
      author: "Dr. Sarah Robotics",
      role: "Robotics Researcher",
      avatar: "🤖",
    },
    {
      quote: "As a robotics engineer, I've found the integrated approach of this platform incredibly valuable. The documentation is comprehensive and the examples are practical.",
      author: "Alex Automation",
      role: "Automation Engineer",
      avatar: "⚙️",
    },
    {
      quote: "Building complex robotic systems has never been easier with the comprehensive modules and AI integration. The platform accelerates development timelines significantly.",
      author: "Jordan Innovator",
      role: "Tech Lead",
      avatar: "🚀",
    },
  ];

  return (
    <section className={styles.testimonialsSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          What Experts Say
        </Heading>
        <p className={styles.sectionSubtitle}>
          Trusted by robotics professionals worldwide
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

function ShowcaseSection() {
  const showcases = [
    {
      title: "Autonomous Mobile Robot",
      description: "Navigation and manipulation in complex environments",
      image: "https://via.placeholder.com/400x200/4f46e5/white?text=AMR"
    },
    {
      title: "Humanoid Robot Control",
      description: "AI-driven locomotion and balance systems",
      image: "https://via.placeholder.com/400x200/ec4899/white?text=HUMANOID"
    },
    {
      title: "Manipulation Tasks",
      description: "Precision grasping and object manipulation",
      image: "https://via.placeholder.com/400x200/0ea5e9/white?text=MANIPULATION"
    },
    {
      title: "Simulation Environments",
      description: "High-fidelity physics and sensor simulation",
      image: "https://via.placeholder.com/400x200/10b981/white?text=SIMULATION"
    }
  ];

  return (
    <section className={styles.showcaseSection}>
      <div className="container">
        <Heading as="h2" className={styles.sectionTitle}>
          Real World Applications
        </Heading>
        <p className={styles.sectionSubtitle}>
          See how our platform is used in actual robotic applications
        </p>

        <div className={styles.showcaseGrid}>
          {showcases.map((showcase, idx) => (
            <div key={idx} className={styles.showcaseCard}>
              <InteractiveCard variant="glass" accentColor={idx % 2 === 0 ? 'cyan' : 'purple'}>
                <div className={styles.showcaseImage}>
                  <img src={showcase.image} alt={showcase.title} />
                </div>
                <h3 className={styles.showcaseTitle}>{showcase.title}</h3>
                <p className={styles.showcaseDescription}>{showcase.description}</p>
              </InteractiveCard>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Robotics Platform - Advanced robotics development with AI integration">
      <HomepageHeader />
      <main className={styles.mainContent}>
        <StatsSection />
        <ModulesSection />
        <ShowcaseSection />
        <TestimonialsSection />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
