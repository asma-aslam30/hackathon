/**
 * Interactive UI Script for Physical AI & Robotics Documentation
 * 
 * This script enhances the UI with modern interactive elements and animations
 */

// Ensure DOM is ready before initializing
document.addEventListener('DOMContentLoaded', function() {
  console.log('🚀 Initializing Interactive UI for Physical AI & Robotics Documentation');
  
  // Initialize all interactive UI elements
  initializeInteractiveUI();
  
  // Initialize animations
  initializeAnimations();
  
  // Add interactive features
  addInteractiveFeatures();
  
  console.log('✨ Interactive UI initialized successfully!');
});

/**
 * Initialize all interactive UI elements
 */
function initializeInteractiveUI() {
  // Initialize interactive cards if they exist
  initializeInteractiveCards();
  
  // Initialize progress indicators
  initializeProgressIndicators();
  
  // Initialize hover effects
  initializeHoverEffects();
  
  // Initialize code block enhancements
  initializeCodeBlockEnhancements();
  
  // Initialize scroll animations
  initializeScrollAnimations();
}

/**
 * Initialize interactive cards with enhanced effects
 */
function initializeInteractiveCards() {
  const cards = document.querySelectorAll('.card, .interactive-element, .module-highlight');
  
  cards.forEach(card => {
    // Add ripple effect to cards on click
    card.addEventListener('click', function(e) {
      if (e.target.closest('a, button')) return; // Don't add ripple to clickable elements inside cards
      
      const ripple = document.createElement('span');
      ripple.classList.add('ripple');
      
      const rect = this.getBoundingClientRect();
      const size = Math.max(rect.width, rect.height);
      const x = e.clientX - rect.left - size / 2;
      const y = e.clientY - rect.top - size / 2;
      
      ripple.style.width = ripple.style.height = `${size}px`;
      ripple.style.left = `${x}px`;
      ripple.style.top = `${y}px`;
      
      this.appendChild(ripple);
      
      setTimeout(() => {
        ripple.remove();
      }, 600);
    });
    
    // Add hover effects
    card.addEventListener('mouseenter', function() {
      this.style.transition = 'all 0.3s ease';
      this.style.transform = 'translateY(-8px) scale(1.02)';
      this.style.boxShadow = '0 25px 50px -12px rgba(0, 0, 0, 0.25)';
    });
    
    card.addEventListener('mouseleave', function() {
      this.style.transform = '';
      this.style.boxShadow = '';
    });
  });
}

/**
 * Initialize progress indicators
 */
function initializeProgressIndicators() {
  const progressBars = document.querySelectorAll('.progress-bar .progress');
  
  progressBars.forEach(bar => {
    // Animate progress bars when they come into view
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          const targetWidth = entry.target.getAttribute('data-target-width') || '100%';
          entry.target.style.width = targetWidth;
          entry.target.style.transition = 'width 1.5s ease-in-out';
          observer.unobserve(entry.target);
        }
      });
    });
    
    observer.observe(bar);
  });
}

/**
 * Initialize hover effects for various elements
 */
function initializeHoverEffects() {
  // Add hover effects to buttons
  const buttons = document.querySelectorAll('.button, button, .button--secondary, .button--primary, .menu__list-item-collapsible');
  
  buttons.forEach(button => {
    button.addEventListener('mouseenter', function() {
      this.style.transform = 'translateY(-3px) scale(1.03)';
      this.style.boxShadow = '0 10px 20px rgba(41, 98, 255, 0.2)';
    });
    
    button.addEventListener('mouseleave', function() {
      this.style.transform = '';
      this.style.boxShadow = '';
    });
  });
  
  // Add hover effects to links
  const links = document.querySelectorAll('a[href^="#"], a[href^="/"]');
  
  links.forEach(link => {
    if (!link.closest('.navbar__link, .pagination-nav__link, .footer__link-item')) {
      link.addEventListener('mouseenter', function() {
        this.style.textShadow = '0 0 8px rgba(41, 98, 255, 0.5)';
        this.style.transition = 'all 0.3s ease';
      });
      
      link.addEventListener('mouseleave', function() {
        this.style.textShadow = '';
      });
    }
  });
}

/**
 * Initialize code block enhancements
 */
function initializeCodeBlockEnhancements() {
  const codeBlocks = document.querySelectorAll('pre[class*="language-"]');
  
  codeBlocks.forEach(block => {
    // Add copy button to code blocks
    if (!block.querySelector('.copy-button')) {
      const copyButton = document.createElement('button');
      copyButton.className = 'copy-button';
      copyButton.innerHTML = 'Copy';
      copyButton.title = 'Copy to clipboard';
      copyButton.style.cssText = `
        position: absolute;
        top: 8px;
        right: 8px;
        padding: 4px 8px;
        background: rgba(0, 0, 0, 0.7);
        color: white;
        border: none;
        border-radius: 4px;
        font-size: 12px;
        cursor: pointer;
        opacity: 0;
        transition: opacity 0.2s ease;
      `;
      
      copyButton.addEventListener('click', function() {
        const code = block.querySelector('code').innerText;
        navigator.clipboard.writeText(code).then(() => {
          const originalText = this.innerText;
          this.innerText = '✓ Copied!';
          setTimeout(() => {
            this.innerText = originalText;
          }, 2000);
        });
      });
      
      block.style.position = 'relative';
      block.appendChild(copyButton);
      
      // Show copy button on hover
      block.addEventListener('mouseenter', function() {
        const copyBtn = this.querySelector('.copy-button');
        if (copyBtn) copyBtn.style.opacity = '1';
      });
      
      block.addEventListener('mouseleave', function() {
        const copyBtn = this.querySelector('.copy-button');
        if (copyBtn) copyBtn.style.opacity = '0';
      });
    }
  });
}

/**
 * Initialize scroll-based animations
 */
function initializeScrollAnimations() {
  // Add scroll-triggered animations for elements
  const animatedElements = document.querySelectorAll('.fade-in-up, .fade-in-down, .tilt-card, .slide-in');
  
  animatedElements.forEach(element => {
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          // Add animation classes when element comes into view
          entry.target.style.opacity = '1';
          entry.target.style.transform = 'translateY(0)';
          
          // Add specific animations based on class
          if (entry.target.classList.contains('fade-in-up')) {
            entry.target.style.transform = 'translateY(0)';
          } else if (entry.target.classList.contains('fade-in-down')) {
            entry.target.style.transform = 'translateY(0)';
          } else if (entry.target.classList.contains('tilt-card')) {
            entry.target.style.transform = 'perspective(1000px) rotateX(0) rotateY(0) translateY(0) scale(1)';
          } else if (entry.target.classList.contains('slide-in')) {
            entry.target.style.transform = 'translateX(0)';
          }
          
          observer.unobserve(entry.target);
        }
      });
    }, {
      threshold: 0.1
    });
    
    observer.observe(element);
  });
}

/**
 * Add additional interactive features
 */
function addInteractiveFeatures() {
  // Add click-to-focus functionality for headings
  const headings = document.querySelectorAll('h1, h2, h3, h4, h5, h6');
  
  headings.forEach(heading => {
    // Add anchor links to headings
    if (!heading.querySelector('.anchor')) {
      const anchor = document.createElement('a');
      anchor.href = `#${heading.id}`;
      anchor.className = 'anchor';
      anchor.innerHTML = '#';
      anchor.style.cssText = `
        margin-left: 8px;
        opacity: 0;
        transition: opacity 0.2s ease;
        text-decoration: none;
        color: inherit;
      `;
      
      heading.style.position = 'relative';
      heading.appendChild(anchor);
      
      // Show anchor on hover
      heading.addEventListener('mouseenter', function() {
        const anchorEl = this.querySelector('.anchor');
        if (anchorEl) anchorEl.style.opacity = '0.6';
      });
      
      heading.addEventListener('mouseleave', function() {
        const anchorEl = this.querySelector('.anchor');
        if (anchorEl) anchorEl.style.opacity = '0';
      });
    }
  });
  
  // Add scroll progress indicator
  addScrollProgressIndicator();
  
  // Add back-to-top button
  addBackToTopButton();
  
  // Add theme toggle enhancements
  addThemeToggleEnhancements();
  
  // Add search enhancements
  addSearchEnhancements();
}

/**
 * Add scroll progress indicator to the top of the page
 */
function addScrollProgressIndicator() {
  const progressDiv = document.createElement('div');
  progressDiv.id = 'scroll-progress';
  progressDiv.style.cssText = `
    position: fixed;
    top: 0;
    left: 0;
    width: 0%;
    height: 3px;
    background: linear-gradient(90deg, #2962ff, #aa00ff);
    z-index: 10000;
    transition: width 0.1s ease;
  `;
  
  document.body.appendChild(progressDiv);
  
  window.addEventListener('scroll', () => {
    const scrollTop = window.pageYOffset;
    const docHeight = document.documentElement.scrollHeight - window.innerHeight;
    const scrollPercent = (scrollTop / docHeight) * 100;
    
    progressDiv.style.width = `${Math.min(scrollPercent, 100)}%`;
  });
}

/**
 * Add back-to-top button
 */
function addBackToTopButton() {
  const button = document.createElement('button');
  button.id = 'back-to-top';
  button.innerHTML = '↑';
  button.style.cssText = `
    position: fixed;
    bottom: 30px;
    right: 30px;
    width: 50px;
    height: 50px;
    border-radius: 50%;
    background: linear-gradient(135deg, #2962ff, #aa00ff);
    color: white;
    border: none;
    font-size: 24px;
    cursor: pointer;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
    opacity: 0;
    visibility: hidden;
    transition: all 0.3s ease;
    z-index: 1000;
  `;
  
  button.addEventListener('click', () => {
    window.scrollTo({
      top: 0,
      behavior: 'smooth'
    });
  });
  
  document.body.appendChild(button);
  
  // Show button when scrolled down
  window.addEventListener('scroll', () => {
    if (window.pageYOffset > 300) {
      button.style.opacity = '1';
      button.style.visibility = 'visible';
    } else {
      button.style.opacity = '0';
      button.style.visibility = 'hidden';
    }
  });
}

/**
 * Add enhancements to theme toggle
 */
function addThemeToggleEnhancements() {
  // Add custom theme toggle with animation
  const themeToggle = document.querySelector('[data-theme-toggle]');
  if (themeToggle) {
    themeToggle.addEventListener('click', function() {
      // Add animation when theme changes
      this.style.transform = 'scale(0.9)';
      setTimeout(() => {
        this.style.transform = '';
      }, 150);
    });
  }
}

/**
 * Add search enhancements
 */
function addSearchEnhancements() {
  // Enhance search functionality if search exists
  const searchBox = document.querySelector('input[placeholder*="Search"], .navbar__search-input');
  if (searchBox) {
    searchBox.addEventListener('focus', function() {
      this.parentElement?.classList.add('search-focused');
    });
    
    searchBox.addEventListener('blur', function() {
      setTimeout(() => {
        this.parentElement?.classList.remove('search-focused');
      }, 200);
    });
  }
}

/**
 * Initialize animations for the page
 */
function initializeAnimations() {
  // Apply initial styles for animated elements
  const animatedElements = document.querySelectorAll('.fade-in-up, .fade-in-down, .tilt-card, .slide-in');
  
  animatedElements.forEach(element => {
    element.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
    
    if (element.classList.contains('fade-in-up')) {
      element.style.opacity = '0';
      element.style.transform = 'translateY(30px)';
    } else if (element.classList.contains('fade-in-down')) {
      element.style.opacity = '0';
      element.style.transform = 'translateY(-30px)';
    } else if (element.classList.contains('tilt-card')) {
      element.style.opacity = '0';
      element.style.transform = 'perspective(1000px) rotateX(5deg) rotateY(5deg) translateY(20px) scale(0.95)';
    } else if (element.classList.contains('slide-in')) {
      element.style.opacity = '0';
      element.style.transform = 'translateX(50px)';
    }
  });
  
  // Initialize 3D effects for tilt cards
  initialize3DEffects();
}

/**
 * Initialize 3D effects for elements
 */
function initialize3DEffects() {
  const tiltElements = document.querySelectorAll('.tilt-card');
  
  tiltElements.forEach(element => {
    element.addEventListener('mousemove', (e) => {
      const rect = element.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      
      const rotateY = (x - centerX) / 25;
      const rotateX = (centerY - y) / 25;
      const glowX = (x / rect.width) * 100;
      const glowY = (y / rect.height) * 100;
      
      element.style.transform = `perspective(1000px) rotateX(${rotateX}deg) rotateY(${rotateY}deg) scale3d(1.05, 1.05, 1.05)`;
      element.style.boxShadow = `${glowX}px ${glowY}px 50px rgba(41, 98, 255, 0.2)`;
    });
    
    element.addEventListener('mouseleave', () => {
      element.style.transform = 'perspective(1000px) rotateX(0) rotateY(0) scale3d(1, 1, 1)';
      element.style.boxShadow = '';
    });
  });
}

// Add a ripple effect CSS if not already present
function addRippleEffectStyles() {
  if (!document.querySelector('#ripple-styles')) {
    const style = document.createElement('style');
    style.id = 'ripple-styles';
    style.textContent = `
      .ripple {
        position: absolute;
        border-radius: 50%;
        background: rgba(255, 255, 255, 0.6);
        transform: scale(0);
        animation: ripple-animation 0.6s linear;
        pointer-events: none;
      }
      
      @keyframes ripple-animation {
        to {
          transform: scale(4);
          opacity: 0;
        }
      }
    `;
    document.head.appendChild(style);
  }
}

// Initialize ripple effect styles
addRippleEffectStyles();

console.log('🔧 Interactive UI Script loaded and ready!');