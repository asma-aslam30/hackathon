import React from 'react';
import Layout from '@theme/Layout';
import Header from '../components/Header/Header';
import Hero from '../components/Hero/Hero';
import FeatureCards from '../components/FeatureCards/FeatureCards';
import Testimonials from '../components/Testimonials/Testimonials';
import CallToAction from '../components/CallToAction/CallToAction';
import { homepageContent } from '../components/homepage-content';

function Homepage() {
  const { header, hero, features, testimonials, callToActions } = homepageContent;

  return (
    <Layout
      title={`Hello`}
      description="AI/Spec-Driven Book Project - Transform Your Understanding of AI & Robotics">
      <main className="homepage-content">
        <Header
          title={header.title}
          logoUrl={header.logoUrl}
          navigationItems={header.navigationItems}
          ctaButton={header.ctaButton}
        />

        <Hero
          title={hero.title}
          subtitle={hero.subtitle}
          description={hero.description}
          imageUrl={hero.imageUrl}
          slides={hero.slides}
          primaryCta={hero.primaryCta}
          secondaryCta={hero.secondaryCta}
        />

        <FeatureCards features={features} />

        <Testimonials testimonials={testimonials} />

        <CallToAction actions={callToActions} />
      </main>
    </Layout>
  );
}

export default Homepage;