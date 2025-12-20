# Component Interface Contract: Docusaurus Landing Page

## Overview
This document defines the interface contracts for the custom landing page components.

## Header Component Contract

### Interface
```typescript
interface HeaderProps {
  title: string;
  logoUrl?: string;
  navigationItems: NavigationItem[];
  ctaButton?: CallToAction;
}

interface NavigationItem {
  label: string;
  url: string;
  type: 'internal' | 'external';
}

interface CallToAction {
  text: string;
  url: string;
  type: 'primary' | 'secondary' | 'link';
  target?: '_self' | '_blank';
}
```

### Expected Behavior
- Renders a responsive header with project title
- Displays navigation items horizontally on desktop, as mobile menu on small screens
- Includes optional CTA button
- Maintains consistent styling across different screen sizes

## Hero Component Contract

### Interface
```typescript
interface HeroProps {
  title: string;
  subtitle: string;
  description: string;
  imageUrl?: string;
  slides?: Slide[];
  primaryCta: CallToAction;
  secondaryCta?: CallToAction;
}

interface Slide {
  title: string;
  subtitle: string;
  description: string;
  imageUrl?: string;
  cta?: CallToAction;
}
```

### Expected Behavior
- Renders a prominent hero section with main headline
- Supports carousel functionality if multiple slides provided
- Includes primary and optional secondary CTA buttons
- Responsive design that works on all screen sizes

## FeatureCards Component Contract

### Interface
```typescript
interface FeatureCardsProps {
  features: FeatureCard[];
}

interface FeatureCard {
  id: string;
  title: string;
  description: string;
  iconUrl?: string;
  link?: string;
}
```

### Expected Behavior
- Displays features in a responsive grid layout
- Each card contains title, description, and optional icon
- Cards are clickable if link is provided
- Layout adjusts based on screen size (1 column on mobile, multiple on desktop)

## Testimonials Component Contract

### Interface
```typescript
interface TestimonialsProps {
  testimonials: Testimonial[];
}

interface Testimonial {
  id: string;
  quote: string;
  author: string;
  role: string;
  company?: string;
  avatarUrl?: string;
}
```

### Expected Behavior
- Displays testimonials in a carousel or grid format
- Each testimonial shows quote, author, and optional role/company
- Responsive layout that works on all screen sizes
- Accessible navigation controls

## CallToAction Component Contract

### Interface
```typescript
interface CallToActionsProps {
  actions: CallToAction[];
}

interface CallToAction {
  id: string;
  text: string;
  url: string;
  type: 'primary' | 'secondary' | 'link';
  target?: '_self' | '_blank';
}
```

### Expected Behavior
- Renders call-to-action buttons with appropriate styling
- Primary buttons are more prominent than secondary
- Links open in specified target window
- Proper hover and focus states for accessibility