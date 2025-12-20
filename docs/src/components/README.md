# Homepage Components Documentation

This directory contains the custom components for the Docusaurus homepage, designed to create a professional landing page for the AI/Spec-Driven Book project.

## Component Structure

### Header Component
- **File**: `Header/Header.js`
- **Purpose**: Navigation header with logo, navigation items, and CTA
- **Props**:
  - `title`: Site title
  - `logoUrl`: Optional logo image URL
  - `navigationItems`: Array of navigation items with label, URL, and type
  - `ctaButton`: Optional call-to-action button

### Hero Component
- **File**: `Hero/Hero.js`
- **Purpose**: Prominent hero section with title, description, and CTAs
- **Features**:
  - Supports single slide or carousel mode
  - Auto-rotating slides with manual controls
  - Responsive design
- **Props**: Title, subtitle, description, image URL, slides array, CTAs

### FeatureCards Component
- **File**: `FeatureCards/FeatureCards.js`
- **Purpose**: Display feature highlights in a responsive grid
- **Features**:
  - Responsive grid layout
  - Hover effects
  - Icon support
- **Props**: Array of feature objects with id, title, description, icon URL, and link

### Testimonials Component
- **File**: `Testimonials/Testimonials.js`
- **Purpose**: Display user testimonials, with optional carousel
- **Features**:
  - Carousel functionality for multiple testimonials
  - Responsive design
  - Avatar support
- **Props**: Array of testimonial objects

### CallToAction Component
- **File**: `CallToAction/CallToAction.js`
- **Purpose**: Prominent call-to-action section
- **Features**:
  - Multiple button support
  - Responsive layout
- **Props**: Array of action objects

## CSS Scoping Strategy

The styling uses a BEM methodology with specific class prefixes to ensure CSS isolation:

- All classes use `homepage-` prefix to prevent style leakage
- Specific selectors target only the homepage content
- CSS reset applied at component level to prevent conflicts with Docusaurus default styles
- High specificity selectors ensure styles don't affect documentation pages

## Implementation Notes

1. **Component Composition**: The main `index.js` page composes all these components to create the complete homepage.

2. **Content Management**: Content is managed through the `homepage-content.js` file which follows the data model structure.

3. **Responsive Design**: All components include responsive design patterns using CSS Grid and Flexbox.

4. **Accessibility**: All components include proper focus states and ARIA attributes.

5. **Performance**: Components are lightweight and optimized for fast loading.

## File Structure

```
components/
├── Header/
│   ├── Header.js          # Header component implementation
│   └── header.css         # Header specific styles
├── Hero/
│   ├── Hero.js            # Hero component implementation
│   └── hero.css           # Hero specific styles
├── FeatureCards/
│   ├── FeatureCards.js    # Feature cards component implementation
│   └── featurecards.css   # Feature cards specific styles
├── Testimonials/
│   ├── Testimonials.js    # Testimonials component implementation
│   └── testimonials.css   # Testimonials specific styles
├── CallToAction/
│   ├── CallToAction.js    # CTA component implementation
│   └── calltoaction.css   # CTA specific styles
├── homepage-content.js    # Content data following data model
└── README.md             # This documentation
```

## Usage

To use these components in other parts of the site, import them as follows:

```javascript
import Header from './components/Header/Header';
import Hero from './components/Hero/Hero';
// ... etc
```

## Testing Notes

- Components should be tested for proper CSS isolation on documentation pages
- Responsive behavior should be verified on mobile, tablet, and desktop
- Accessibility features should be tested with keyboard navigation and screen readers
- Cross-browser compatibility should be verified