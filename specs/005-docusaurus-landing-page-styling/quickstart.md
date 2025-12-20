# Quickstart: Docusaurus Homepage Component Styling

## Overview
This guide provides the essential steps to implement the custom homepage React component with dedicated CSS styling for the Docusaurus site, ensuring CSS selectors do not affect docs or book pages.

## Prerequisites
- Node.js 18+ installed
- Docusaurus CLI installed (`npm install -g @docusaurus/core`)
- Access to the project repository

## Setup Steps

### 1. Clone and Install Dependencies
```bash
git clone <repository-url>
cd <repository-name>
cd docs
npm install
```

### 2. Create Custom Components Directory
```bash
mkdir -p src/components/{Header,Hero,FeatureCards,Testimonials,CallToAction}
```

### 3. Create Custom CSS File
```bash
mkdir -p src/css
touch src/css/custom.css
```

### 4. Create Homepage Component
```bash
mkdir -p src/pages
touch src/pages/index.js
```

## Implementation Steps

### 1. Add Custom CSS
In `src/css/custom.css`, add your scoped CSS styles:
- Use highly specific selectors that target only the homepage
- Use class names with `homepage-` prefix to ensure scoping
- Implement responsive design with media queries
- Follow accessibility best practices (contrast, focus states)

### 2. Create React Components
Create the following components in their respective directories:
- `src/components/Header/Header.js` - Custom header with navigation
- `src/components/Hero/Hero.js` - Hero section with carousel functionality
- `src/components/FeatureCards/FeatureCards.js` - Feature highlight cards
- `src/components/Testimonials/Testimonials.js` - Testimonial cards
- `src/components/CallToAction/CallToAction.js` - CTA buttons

### 3. Update Docusaurus Configuration
In `docusaurus.config.ts`, ensure custom styles are loaded:
```js
stylesheets: [
  {
    href: '/css/custom.css',
    type: 'text/css',
  },
],
```

### 4. Create Homepage Component
In `src/pages/index.js`, import and use your custom components to build the homepage layout with responsive design for all sections.

### 5. Test Styling Isolation
- Verify that styling does not leak to other pages
- Test by navigating to book chapter pages and confirming default styling remains
- Use browser dev tools to verify CSS scope

## Testing Commands
```bash
# Start development server
npm start

# Build for production
npm run build

# Serve built site locally
npm run serve
```

## Verification
1. Visit `http://localhost:3000` during development
2. Confirm all homepage elements are styled correctly
3. Verify book chapters still use default Docusaurus styling (no custom CSS applied)
4. Test responsive layout for all homepage sections
5. Test accessibility features (keyboard navigation, screen readers)
6. Check that CSS selectors are properly scoped and don't affect other pages