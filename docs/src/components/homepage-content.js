// Placeholder content data based on data-model.md
export const homepageContent = {
  header: {
    title: "AI/Spec-Driven Book Project",
    logoUrl: null,
    navigationItems: [
      { label: "Home", url: "/", type: "internal" },
      { label: "Book", url: "/docs/physical-ai-robotics/introduction", type: "internal" },
      { label: "AI Chatbot", url: "/chatbot", type: "internal" },
      { label: "GitHub", url: "https://github.com", type: "external" }
    ],
    ctaButton: {
      text: "Get Started",
      url: "/docs/physical-ai-robotics/introduction",
      type: "primary",
      target: "_self"
    }
  },
  hero: {
    title: "Transform Your Understanding of AI & Robotics",
    subtitle: "Spec-Driven Development for the Modern Age",
    description: "Explore comprehensive guides, practical examples, and cutting-edge techniques for building AI-powered systems with a focus on specification-driven approaches.",
    imageUrl: "/img/hero-illustration.svg", // Placeholder
    primaryCta: {
      id: "hero-primary-cta",
      text: "Read the Book",
      url: "/docs/physical-ai-robotics/introduction",
      type: "primary",
      target: "_self"
    },
    secondaryCta: {
      id: "hero-secondary-cta",
      text: "Ask AI Chatbot",
      url: "/chatbot",
      type: "secondary",
      target: "_self"
    },
    slides: [
      {
        title: "Master Spec-Driven Development",
        subtitle: "AI & Robotics Edition",
        description: "Learn how to build robust AI systems using specification-driven approaches that ensure quality and maintainability.",
        imageUrl: "/img/hero-slide-1.svg",
        cta: {
          id: "slide1-cta",
          text: "Start Learning",
          url: "/docs/physical-ai-robotics/introduction",
          type: "primary",
          target: "_self"
        }
      },
      {
        title: "Cutting-Edge AI Techniques",
        subtitle: "Applied to Robotics",
        description: "Discover the latest techniques in AI and robotics, with practical examples and real-world applications.",
        imageUrl: "/img/hero-slide-2.svg",
        cta: {
          id: "slide2-cta",
          text: "Explore Examples",
          url: "/docs/physical-ai-robotics/module-1-ros2/ros2-fundamentals/installation",
          type: "primary",
          target: "_self"
        }
      }
    ]
  },
  features: [
    {
      id: "feature-1",
      title: "Comprehensive Guides",
      description: "Detailed explanations of complex topics with practical examples and best practices.",
      iconUrl: "/img/icon-guide.svg", // Placeholder
      link: "/docs/physical-ai-robotics/introduction"
    },
    {
      id: "feature-2",
      title: "Interactive Examples",
      description: "Hands-on examples you can run and modify to deepen your understanding.",
      iconUrl: "/img/icon-example.svg", // Placeholder
      link: "/docs/physical-ai-robotics/module-1-ros2/ros2-fundamentals/workspace-setup"
    },
    {
      id: "feature-3",
      title: "AI Integration",
      description: "Learn how to integrate AI into your systems with practical patterns and techniques.",
      iconUrl: "/img/icon-ai.svg", // Placeholder
      link: "/docs/physical-ai-robotics/module-3-isaac-brain/overview"
    }
  ],
  testimonials: [
    {
      id: "testimonial-1",
      quote: "This book completely transformed how I approach AI development. The spec-driven methodology has made my projects more reliable and maintainable.",
      author: "Alex Johnson",
      role: "Senior AI Engineer",
      company: "TechCorp",
      avatarUrl: "/img/avatar-alex.jpg" // Placeholder
    },
    {
      id: "testimonial-2",
      quote: "The practical examples and clear explanations made complex concepts accessible. Highly recommended for anyone working with AI systems.",
      author: "Sarah Chen",
      role: "Robotics Researcher",
      company: "Innovation Labs",
      avatarUrl: "/img/avatar-sarah.jpg" // Placeholder
    }
  ],
  callToActions: [
    {
      id: "cta-read-book",
      text: "Read the Book",
      url: "/docs/physical-ai-robotics/introduction",
      type: "primary",
      target: "_self"
    },
    {
      id: "cta-ai-chatbot",
      text: "Ask AI Chatbot",
      url: "/chatbot",
      type: "secondary",
      target: "_self"
    }
  ]
};