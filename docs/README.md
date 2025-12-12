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
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Content Structure

The documentation is organized into modules:
1. Introduction to Physical AI
2. ROS 2 - The Robotic Nervous System
3. Digital Twin - Gazebo & Unity Simulation
4. AI-Robot Brain - NVIDIA Isaac
5. Vision-Language-Action Systems
6. Capstone Project - Autonomous Humanoid

Each module includes code examples, diagrams, and practical exercises that integrate with Model Context Protocol (MCP) and Context7 for enhanced documentation access.
