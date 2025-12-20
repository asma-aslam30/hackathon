import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './hero.css';

const Hero = ({ title, subtitle, description, imageUrl, slides = [], primaryCta, secondaryCta }) => {
  const [currentSlide, setCurrentSlide] = useState(0);

  // Use slides if provided, otherwise use single slide data
  const heroSlides = slides.length > 0 ? slides : [{
    title: title,
    subtitle: subtitle,
    description: description,
    imageUrl: imageUrl,
    cta: primaryCta
  }];

  const goToSlide = (index) => {
    setCurrentSlide(index);
  };

  const nextSlide = () => {
    setCurrentSlide((prev) => (prev + 1) % heroSlides.length);
  };

  const prevSlide = () => {
    setCurrentSlide((prev) => (prev - 1 + heroSlides.length) % heroSlides.length);
  };

  React.useEffect(() => {
    const interval = setInterval(() => {
      nextSlide();
    }, 5000); // Auto-advance every 5 seconds

    return () => clearInterval(interval);
  }, [heroSlides.length]);

  return (
    <section className="homepage-hero">
      <div className="homepage-hero__container">
        {heroSlides.length > 1 ? (
          // Carousel mode
          <div className="homepage-hero__carousel">
            {heroSlides.map((slide, index) => (
              <div
                key={index}
                className={clsx(
                  'homepage-hero__slide',
                  index === currentSlide ? 'homepage-hero__slide--active' : 'homepage-hero__slide--inactive'
                )}
              >
                <div className="homepage-hero__content">
                  <h1 className="homepage-hero__title">{slide.title}</h1>
                  <h2 className="homepage-hero__subtitle">{slide.subtitle}</h2>
                  <p className="homepage-hero__description">{slide.description}</p>

                  <div className="homepage-hero__cta-container">
                    {slide.cta && (
                      <Link
                        to={slide.cta.url}
                        className={clsx(
                          'homepage-hero__cta-button',
                          `homepage-hero__cta-button--${slide.cta.type || 'primary'}`
                        )}
                        target={slide.cta.target || '_self'}
                      >
                        {slide.cta.text}
                      </Link>
                    )}
                    {secondaryCta && (
                      <Link
                        to={secondaryCta.url}
                        className={clsx(
                          'homepage-hero__cta-button',
                          `homepage-hero__cta-button--${secondaryCta.type || 'secondary'}`
                        )}
                        target={secondaryCta.target || '_self'}
                      >
                        {secondaryCta.text}
                      </Link>
                    )}
                  </div>
                </div>

                {slide.imageUrl && (
                  <div className="homepage-hero__image-container">
                    <img
                      src={slide.imageUrl}
                      alt={slide.title}
                      className="homepage-hero__image"
                    />
                  </div>
                )}
              </div>
            ))}

            {/* Carousel controls */}
            <div className="homepage-hero__controls">
              <button
                className="homepage-hero__control homepage-hero__control--prev"
                onClick={prevSlide}
                aria-label="Previous slide"
              >
                &lt;
              </button>
              <button
                className="homepage-hero__control homepage-hero__control--next"
                onClick={nextSlide}
                aria-label="Next slide"
              >
                &gt;
              </button>
            </div>

            {/* Carousel indicators */}
            <div className="homepage-hero__indicators">
              {heroSlides.map((_, index) => (
                <button
                  key={index}
                  className={clsx(
                    'homepage-hero__indicator',
                    index === currentSlide ? 'homepage-hero__indicator--active' : ''
                  )}
                  onClick={() => goToSlide(index)}
                  aria-label={`Go to slide ${index + 1}`}
                />
              ))}
            </div>
          </div>
        ) : (
          // Single slide mode
          <div className="homepage-hero__single">
            <div className="homepage-hero__content">
              <h1 className="homepage-hero__title">{title}</h1>
              <h2 className="homepage-hero__subtitle">{subtitle}</h2>
              <p className="homepage-hero__description">{description}</p>

              <div className="homepage-hero__cta-container">
                {primaryCta && (
                  <Link
                    to={primaryCta.url}
                    className={clsx(
                      'homepage-hero__cta-button',
                      `homepage-hero__cta-button--${primaryCta.type || 'primary'}`
                    )}
                    target={primaryCta.target || '_self'}
                  >
                    {primaryCta.text}
                  </Link>
                )}
                {secondaryCta && (
                  <Link
                    to={secondaryCta.url}
                    className={clsx(
                      'homepage-hero__cta-button',
                      `homepage-hero__cta-button--${secondaryCta.type || 'secondary'}`
                    )}
                    target={secondaryCta.target || '_self'}
                  >
                    {secondaryCta.text}
                  </Link>
                )}
              </div>
            </div>

            {imageUrl && (
              <div className="homepage-hero__image-container">
                <img
                  src={imageUrl}
                  alt={title}
                  className="homepage-hero__image"
                />
              </div>
            )}
          </div>
        )}
      </div>
    </section>
  );
};

export default Hero;