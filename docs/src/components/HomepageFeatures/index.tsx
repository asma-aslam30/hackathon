import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import InteractiveCard from '@site/src/components/InteractiveCard';
import { BentoGrid, BentoItem } from '@site/src/components/BentoGrid';
import styles from './styles.module.css';

// SVG Icon Components
const LightningIcon = () => (
  <svg className={styles.featureIconSvg} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M13 2L3 14h8l-1 8 10-12h-8l1-8z" />
  </svg>
);

const SparklesIcon = () => (
  <svg className={styles.featureIconSvg} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M12 3v18m9-9H3m15.364 6.364L5.636 5.636m12.728 0L5.636 18.364" />
  </svg>
);

const DevicesIcon = () => (
  <svg className={styles.featureIconSvg} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <rect x="2" y="3" width="20" height="14" rx="2" />
    <path d="M8 21h8m-4-4v4" />
  </svg>
);

const CodeIcon = () => (
  <svg className={styles.featureIconSvg} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <polyline points="16 18 22 12 16 6" />
    <polyline points="8 6 2 12 8 18" />
  </svg>
);

const AccessibleIcon = () => (
  <svg className={styles.featureIconSvg} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <circle cx="12" cy="12" r="10" />
    <path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3m.08 4h.01" />
  </svg>
);

const HeartIcon = () => (
  <svg className={styles.featureIconSvg} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M20.84 4.61a5.5 5.5 0 0 0-7.78 0L12 5.67l-1.06-1.06a5.5 5.5 0 0 0-7.78 7.78l1.06 1.06L12 21.23l7.78-7.78 1.06-1.06a5.5 5.5 0 0 0 0-7.78z" />
  </svg>
);

type FeatureItem = {
  title: string;
  description: ReactNode;
  Icon: () => ReactNode;
  variant: 'glass' | '3d' | 'gradient' | 'magnetic' | 'neon';
  accentColor: 'cyan' | 'purple' | 'pink';
  size?: 'small' | 'medium' | 'large' | 'wide' | 'tall';
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Lightning Fast',
    Icon: LightningIcon,
    variant: 'neon',
    accentColor: 'cyan',
    size: 'medium',
    description: (
      <>
        Built for speed with optimized performance and instant page loads.
        Experience blazing-fast navigation and smooth interactions.
      </>
    ),
  },
  {
    title: 'Modern Design',
    Icon: SparklesIcon,
    variant: 'gradient',
    accentColor: 'purple',
    size: 'medium',
    description: (
      <>
        Stunning cyber-minimalist aesthetics with glassmorphism effects,
        animated gradients, and interactive elements.
      </>
    ),
  },
  {
    title: 'Fully Responsive',
    Icon: DevicesIcon,
    variant: '3d',
    accentColor: 'pink',
    size: 'medium',
    description: (
      <>
        Perfect on any device. Seamlessly adapts from mobile to desktop
        with fluid layouts and touch-optimized interactions.
      </>
    ),
  },
  {
    title: 'Developer Friendly',
    Icon: CodeIcon,
    variant: 'glass',
    accentColor: 'cyan',
    size: 'medium',
    description: (
      <>
        Easy to customize and extend. Built with React and TypeScript
        for maximum flexibility and type safety.
      </>
    ),
  },
  {
    title: 'Accessible',
    Icon: AccessibleIcon,
    variant: 'magnetic',
    accentColor: 'purple',
    size: 'medium',
    description: (
      <>
        WCAG compliant with keyboard navigation, screen reader support,
        and respect for motion preferences.
      </>
    ),
  },
  {
    title: 'Open Source',
    Icon: HeartIcon,
    variant: 'neon',
    accentColor: 'pink',
    size: 'medium',
    description: (
      <>
        Free and open source. Join our community and contribute to
        building the future of documentation.
      </>
    ),
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <Heading as="h2" className={styles.featuresTitle}>
          Why Choose Us
        </Heading>
        <p className={styles.featuresSubtitle}>
          Experience the next generation of documentation with cutting-edge design and technology
        </p>

        <BentoGrid>
          {FeatureList.map((feature, idx) => (
            <BentoItem key={idx} size={feature.size}>
              <InteractiveCard
                variant={feature.variant}
                title={feature.title}
                description={feature.description}
                icon={<feature.Icon />}
                accentColor={feature.accentColor}
              />
            </BentoItem>
          ))}
        </BentoGrid>
      </div>
    </section>
  );
}
