# Physical AI & Humanoid Robotics Documentation

This website contains comprehensive documentation for the "Physical AI & Humanoid Robotics" textbook, built using [Docusaurus](https://docusaurus.io/), a modern static website generator.

## About This Documentation

This documentation covers the complete curriculum for developing autonomous humanoid robots with a focus on:
- ROS 2 as the robotic nervous system
- Advanced simulation with Gazebo and Unity
- NVIDIA Isaac as the AI-robot brain
- Vision-Language-Action (VLA) systems for multimodal interaction
- Complete capstone project: The Autonomous Humanoid

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Troubleshooting Common Issues

### Missing Images

If images are not displaying on the homepage, you'll see broken image icons or placeholders. This is expected during initial setup as the demo content references placeholder images that don't exist in the repository.

**Solution**: Create the required image assets or update the content to use available images:

1. **Add placeholder images** (for development):
   ```bash
   mkdir -p static/img
   # Create placeholder SVG images for development
   echo '<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="#e2e8f0"/><text x="200" y="150" font-family="Arial" font-size="16" text-anchor="middle" fill="#4a5568">Homepage Image</text></svg>' > static/img/hero-illustration.svg
   echo '<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="#feb2b2"/><text x="200" y="150" font-family="Arial" font-size="16" text-anchor="middle" fill="#4a5568">Slide 1</text></svg>' > static/img/hero-slide-1.svg
   echo '<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="#fed7d7"/><text x="200" y="150" font-family="Arial" font-size="16" text-anchor="middle" fill="#4a5568">Slide 2</text></svg>' > static/img/hero-slide-2.svg
   echo '<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"><circle cx="50" cy="50" r="40" fill="#90cdf4"/></svg>' > static/img/icon-guide.svg
   echo '<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"><circle cx="50" cy="50" r="40" fill="#68d391"/></svg>' > static/img/icon-example.svg
   echo '<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"><circle cx="50" cy="50" r="40" fill="#fbbf24"/></svg>' > static/img/icon-ai.svg
   ```

2. **Update content with valid image paths**:
   Edit `src/components/homepage-content.js` to use images from the `static/img/` directory.

### Development Commands

- `npm start` - Starts the development server with hot reloading
- `npm run build` - Builds the static site for production
- `npm run serve` - Serves the built site locally (after running build)
- `npm run docusaurus clear` - Clears the build cache if you encounter issues

### Custom Components Location

The custom homepage components are located in:
- `src/components/Header/` - Navigation header
- `src/components/Hero/` - Hero section with carousel
- `src/components/FeatureCards/` - Feature highlights
- `src/components/Testimonials/` - Testimonials section
- `src/components/CallToAction/` - Call-to-action section
- `src/pages/index.js` - Main homepage composition
- `src/css/custom.css` - Custom styling with CSS scoping

### CSS Scoping Information

The homepage styling is specifically scoped to prevent affecting documentation pages:
- All homepage components use `homepage-` prefixed CSS classes
- Styles are isolated to prevent leaking to other pages
- Responsive design is implemented using CSS Grid and Flexbox

## Environment Variables

If your site requires environment variables, create a `.env` file in the `docs/` directory:
```bash
# Example .env file
CUSTOM_VARIABLE=value
```

## Troubleshooting Tips

1. **If the page doesn't refresh after changes**: Try refreshing the browser or restarting the development server with `Ctrl+C` and `npm start`.

2. **If you see CSS issues**: Clear the cache with `npm run docusaurus clear` and restart the server.

3. **If links don't work properly**: Check that all navigation links in `src/components/homepage-content.js` point to valid routes in your Docusaurus site.

4. **For performance issues**: The site should load within 3 seconds. If experiencing slow load times, check your custom CSS for overly complex selectors.

## Content Structure

The documentation is organized into modules:
1. Introduction to Physical AI
2. ROS 2 - The Robotic Nervous System
3. Digital Twin - Gazebo & Unity Simulation
4. AI-Robot Brain - NVIDIA Isaac
5. Vision-Language-Action Systems
6. Capstone Project - Autonomous Humanoid

Each module includes code examples, diagrams, and practical exercises that integrate with Model Context Protocol (MCP) and Context7 for enhanced documentation access.
