/**
 * Interactive UI Functions for Physical AI & Robotics Documentation
 * Enhanced user interface interactions and animations
 */

// Initialize interactive elements when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
  initializeInteractiveElements();
  initializeAnimations();
});

// Main initialization function
function initializeInteractiveElements() {
  // Initialize cybernetic buttons
  initializeCyberButtons();
  
  // Initialize robotic path visualizations
  initializeRobotPathVisuals();
  
  // Initialize sensor visualizations
  initializeSensorVisuals();
  
  // Initialize neural network visualizations
  initializeNeuralVisuals();
  
  // Initialize progress indicators
  initializeProgressIndicators();
  
  // Initialize hover effects
  initializeHoverEffects();
  
  // Initialize terminal blocks
  initializeTerminalBlocks();
}

// Initialize enhanced cybernetic buttons
function initializeCyberButtons() {
  const btns = document.querySelectorAll('.robotic-btn');
  btns.forEach(btn => {
    btn.addEventListener('click', function(e) {
      e.preventDefault();
      
      // Add ripple effect
      const ripple = document.createElement('span');
      ripple.classList.add('ripple');
      this.appendChild(ripple);
      
      // Remove ripple after animation
      setTimeout(() => {
        ripple.remove();
      }, 600);
    });
  });
}

// Initialize robotic path visualizations
function initializeRobotPathVisuals() {
  const paths = document.querySelectorAll('.robot-path');
  paths.forEach(path => {
    // Animate path drawing
    const length = path.getTotalLength();
    path.style.strokeDasharray = length;
    path.style.strokeDashoffset = length;
    
    // Draw path when element is visible
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          path.style.transition = 'stroke-dashoffset 2s ease-in-out';
          path.style.strokeDashoffset = '0';
          observer.unobserve(path);
        }
      });
    });
    
    observer.observe(path);
  });
}

// Initialize sensor visualizations
function initializeSensorVisuals() {
  const dots = document.querySelectorAll('.sensor-dot');
  dots.forEach(dot => {
    // Add pulsating effect
    dot.style.animation = 'pulse-glow 2s infinite alternate';
  });
}

// Initialize neural network visualizations
function initializeNeuralVisuals() {
  const nodes = document.querySelectorAll('.neural-node');
  nodes.forEach(node => {
    // Add neural activation effect
    setInterval(() => {
      const brightness = Math.random() * 0.5 + 0.5; // 0.5 to 1.0
      node.style.opacity = brightness;
    }, 300);
  });
}

// Initialize progress indicators
function initializeProgressIndicators() {
  const progressBars = document.querySelectorAll('.progress-cyber-fill');
  progressBars.forEach(bar => {
    // Animate progress bars when they come into view
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          const progressValue = bar.getAttribute('data-progress') || '50';
          bar.style.setProperty('--progress-value', `${progressValue}%`);
          observer.unobserve(bar);
        }
      });
    });
    
    observer.observe(bar);
  });
}

// Initialize hover effects
function initializeHoverEffects() {
  // Add hover effects to cards and other elements
  const hoverables = document.querySelectorAll('.trigger-hover-grow, .trigger-hover-lift, .trigger-hover-shadow');
  hoverables.forEach(element => {
    element.addEventListener('mouseenter', function() {
      this.style.transition = 'all 0.3s ease';
    });
    
    element.addEventListener('mouseleave', function() {
      this.style.transform = 'none';
      this.style.boxShadow = 'none';
    });
  });
}

// Initialize terminal-style code blocks
function initializeTerminalBlocks() {
  const terminals = document.querySelectorAll('.terminal-block');
  terminals.forEach(term => {
    // Add typing animation to terminal content
    const content = term.querySelector('.terminal-content');
    if (content) {
      const text = content.textContent;
      content.textContent = '';
      
      let i = 0;
      const speed = 20; // ms per character
      
      const typeWriter = () => {
        if (i < text.length) {
          content.textContent += text.charAt(i);
          i++;
          setTimeout(typeWriter, speed);
        }
      };
      
      // Start typing when element is visible
      const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
          if (entry.isIntersecting) {
            setTimeout(typeWriter, 500); // Delay to allow for page load
            observer.unobserve(term);
          }
        });
      });
      
      observer.observe(term);
    }
  });
}

// Initialize animations that should happen on page load
function initializeAnimations() {
  // Fade-in animations for elements
  const fadeElements = document.querySelectorAll('.fade-in-up, .fade-in-down');
  fadeElements.forEach(el => {
    // Apply fade-in animation when element comes into view
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.style.opacity = '1';
          entry.target.style.transform = 'translateY(0)';
          observer.unobserve(entry.target);
        }
      });
    });
    
    observer.observe(el);
  });
  
  // Apply initial styles for fade-in elements
  fadeElements.forEach(el => {
    el.style.opacity = '0';
    el.style.transform = 'translateY(30px)';
    el.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
  });
  
  // Initialize grid animations
  initializeGridAnimations();
}

// Initialize grid layout animations
function initializeGridAnimations() {
  const gridItems = document.querySelectorAll('.grid-item');
  gridItems.forEach((item, index) => {
    // Set initial state
    item.style.opacity = '0';
    item.style.transform = 'translateY(50px)';
    item.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
    item.style.transitionDelay = `${index * 0.1}s`;
    
    // Animate when visible
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.style.opacity = '1';
          entry.target.style.transform = 'translateY(0)';
          observer.unobserve(entry.target);
        }
      });
    });
    
    observer.observe(item);
  });
}

// Utility function to create ripple effect
function createRipple(event) {
  const btn = event.currentTarget;
  const circle = document.createElement("span");
  const diameter = Math.max(btn.clientWidth, btn.clientHeight);
  const radius = diameter / 2;

  circle.style.width = circle.style.height = `${diameter}px`;
  circle.style.left = `${event.clientX - btn.getBoundingClientRect().left - radius}px`;
  circle.style.top = `${event.clientY - btn.getBoundingClientRect().top - radius}px`;
  circle.classList.add("ripple");

  const ripple = btn.getElementsByClassName("ripple")[0];
  if (ripple) ripple.remove();
  
  btn.appendChild(circle);
}

// Function to create interactive hover effects
function createHoverEffect(element, effectType) {
  switch(effectType) {
    case 'glow':
      element.addEventListener('mouseenter', () => {
        element.style.boxShadow = '0 0 20px rgba(41, 98, 255, 0.6)';
        element.style.transform = 'scale(1.02)';
      });
      element.addEventListener('mouseleave', () => {
        element.style.boxShadow = '';
        element.style.transform = '';
      });
      break;
      
    case 'tilt':
      element.addEventListener('mousemove', (e) => {
        const rect = element.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        const rotateX = (y - centerY) / 10;
        const rotateY = (centerX - x) / 10;
        
        element.style.transform = `perspective(1000px) rotateX(${rotateX}deg) rotateY(${rotateY}deg) scale(1.05)`;
      });
      
      element.addEventListener('mouseleave', () => {
        element.style.transform = '';
      });
      break;
  }
}

// Function to create animated counters
function createAnimatedCounter(element, targetNumber, duration = 2000) {
  let start = 0;
  const increment = targetNumber / (duration / 16);
  
  const updateCounter = () => {
    start += increment;
    if (start < targetNumber) {
      element.textContent = Math.ceil(start);
      setTimeout(updateCounter, 16);
    } else {
      element.textContent = targetNumber;
    }
  };
  
  updateCounter();
}

// Export functions for use in other modules
if (typeof module !== 'undefined' && module.exports) {
  module.exports = {
    initializeInteractiveElements,
    initializeAnimations,
    createRipple,
    createHoverEffect,
    createAnimatedCounter
  };
}

// Add interactive elements to headings
function addInteractiveHeadings() {
  const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
  headings.forEach(heading => {
    if (!heading.classList.contains('interactive-heading')) {
      heading.classList.add('interactive-heading');
      
      // Add click to copy functionality
      heading.addEventListener('click', function() {
        const text = this.textContent;
        navigator.clipboard.writeText(text)
          .then(() => {
            // Visual feedback
            const originalText = this.innerHTML;
            this.innerHTML = `${originalText} ✓`;
            setTimeout(() => {
              this.innerHTML = originalText;
            }, 1000);
          })
          .catch(err => {
            console.error('Failed to copy text: ', err);
          });
      });
    }
  });
}

// Initialize interactive headings after page load
window.addEventListener('load', addInteractiveHeadings);

// Add scroll progress indicator
function addScrollProgressIndicator() {
  const progressBar = document.createElement('div');
  progressBar.className = 'scroll-progress-bar';
  progressBar.style.cssText = `
    position: fixed;
    top: 0;
    left: 0;
    width: 0%;
    height: 3px;
    background: linear-gradient(90deg, #2962ff, #aa00ff);
    z-index: 9999;
    transition: width 0.05s ease;
  `;
  
  document.body.appendChild(progressBar);
  
  window.addEventListener('scroll', () => {
    const scrollTop = window.pageYOffset;
    const docHeight = document.body.offsetHeight;
    const winHeight = window.innerHeight;
    const scrollPercent = scrollTop / (docHeight - winHeight);
    const scrollPercentRounded = Math.round(scrollPercent * 100) / 100;
    
    const progressWidth = scrollPercentRounded * 100;
    progressBar.style.width = `${progressPercentRounded * 100}%`;
  });
}

// Add scroll progress indicator after page load
window.addEventListener('load', addScrollProgressIndicator);