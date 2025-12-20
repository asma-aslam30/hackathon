# Research: Docusaurus Landing Page Homepage Styling

## Overview
This document outlines the research and technical decisions for creating or updating the Docusaurus homepage React component with dedicated custom CSS styling that ensures CSS selectors do not affect docs or book pages. Implementation will focus on responsive layout for all landing page sections while verifying no styling leaks into Markdown chapters.

## Decision: Homepage Component Strategy
**Rationale**: To customize the Docusaurus homepage, we need to create or update the index.js file in the pages directory.

**Approach**:
1. Create/update `docs/src/pages/index.js` as the homepage component
2. Import and compose custom components (Header, Hero, FeatureCards, Testimonials, CallToAction)
3. Structure the component to match the design requirements from the spec
4. Ensure the component integrates properly with Docusaurus

**Alternatives considered**:
- Modifying existing Docusaurus theme: Would be harder to maintain
- Using Docusaurus markdown pages: Would not allow for complex custom components

## Decision: CSS Scoping Strategy
**Rationale**: To ensure custom CSS only affects the landing page and not book chapters, we'll use CSS classes with specific prefixes and Docusaurus' ability to apply custom styles to specific pages.

**Approach**:
1. Create a custom CSS file `docs/src/css/custom.css`
2. Use highly specific selectors that target only the landing page
3. Apply CSS modules or BEM methodology to prevent class name collisions
4. Use the `.main-wrapper` and page-specific classes that Docusaurus provides
5. Implement CSS nesting for better organization and specificity

**Alternatives considered**:
- Global CSS with complex selectors: Risk of affecting other pages
- Inline styles: Harder to maintain and reuse
- CSS-in-JS: Would require additional dependencies, violating "CSS only" constraint

## Decision: CSS File Integration
**Rationale**: The custom CSS file needs to be properly loaded in the Docusaurus application.

**Approach**:
1. Add the CSS file reference in `docusaurus.config.ts` under stylesheets
2. Ensure the CSS file is loaded only when needed
3. Use appropriate CSS loading strategy to avoid conflicts

**Alternatives considered**:
- Importing CSS in individual components: Would lead to multiple HTTP requests
- Using Docusaurus' global styles: Risk of affecting other pages

## Decision: Responsive Layout Implementation
**Rationale**: The layout must be responsive across all device sizes as specified in the requirements.

**Approach**:
1. Use CSS Grid and Flexbox for responsive layouts
2. Implement mobile-first approach with media queries
3. Ensure all sections (header, hero, features, testimonials, CTAs) are responsive
4. Test on multiple screen sizes during development

**Alternatives considered**:
- Fixed-width design: Would not meet responsive requirement
- JavaScript-based responsive design: Would add unnecessary complexity

## Decision: Verification of Styling Isolation
**Rationale**: It's critical that styling doesn't leak to other pages as specified in the requirements.

**Approach**:
1. Use highly specific CSS selectors that only target the homepage
2. Test by navigating to book chapter pages and verifying default styling remains
3. Use browser dev tools to verify CSS scope
4. Implement CSS reset at the component level if needed

**Alternatives considered**:
- Less specific selectors: Risk of affecting other pages
- No verification process: Would not meet requirements

## Implementation Strategy
1. Create or update the homepage React component (`docs/src/pages/index.js`)
2. Create dedicated custom CSS file (`docs/src/css/custom.css`)
3. Implement responsive layout for all sections
4. Test CSS scoping to ensure no leaks to other pages
5. Verify all landing page sections display correctly on different devices