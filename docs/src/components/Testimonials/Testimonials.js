import React, { useState } from 'react';
import clsx from 'clsx';
import './testimonials.css';

const TestimonialCard = ({ id, quote, author, role, company, avatarUrl }) => {
  return (
    <div className="homepage-testimonial-card">
      <div className="homepage-testimonial-card__quote">
        <div className="homepage-testimonial-card__quote-text">
          "{quote}"
        </div>
      </div>
      <div className="homepage-testimonial-card__author">
        {avatarUrl && (
          <img
            src={avatarUrl}
            alt={author}
            className="homepage-testimonial-card__avatar"
          />
        )}
        <div className="homepage-testimonial-card__author-info">
          <div className="homepage-testimonial-card__author-name">{author}</div>
          <div className="homepage-testimonial-card__author-role">
            {role}{company ? `, ${company}` : ''}
          </div>
        </div>
      </div>
    </div>
  );
};

const Testimonials = ({ testimonials = [] }) => {
  const [currentTestimonial, setCurrentTestimonial] = useState(0);

  const nextTestimonial = () => {
    setCurrentTestimonial((prev) => (prev + 1) % testimonials.length);
  };

  const prevTestimonial = () => {
    setCurrentTestimonial((prev) => (prev - 1 + testimonials.length) % testimonials.length);
  };

  React.useEffect(() => {
    if (testimonials.length <= 1) return;

    const interval = setInterval(() => {
      nextTestimonial();
    }, 7000); // Auto-advance every 7 seconds

    return () => clearInterval(interval);
  }, [testimonials.length]);

  return (
    <section className="homepage-testimonials">
      <div className="homepage-testimonials__container">
        <h2 className="homepage-testimonials__title">What Our Users Say</h2>

        {testimonials.length > 1 ? (
          // Carousel mode for multiple testimonials
          <div className="homepage-testimonials__carousel">
            {testimonials.map((testimonial, index) => (
              <div
                key={testimonial.id || index}
                className={clsx(
                  'homepage-testimonials__slide',
                  index === currentTestimonial ? 'homepage-testimonials__slide--active' : 'homepage-testimonials__slide--inactive'
                )}
              >
                <TestimonialCard {...testimonial} />
              </div>
            ))}

            {/* Carousel controls */}
            <div className="homepage-testimonials__controls">
              <button
                className="homepage-testimonials__control homepage-testimonials__control--prev"
                onClick={prevTestimonial}
                aria-label="Previous testimonial"
              >
                &lt;
              </button>
              <button
                className="homepage-testimonials__control homepage-testimonials__control--next"
                onClick={nextTestimonial}
                aria-label="Next testimonial"
              >
                &gt;
              </button>
            </div>

            {/* Carousel indicators */}
            <div className="homepage-testimonials__indicators">
              {testimonials.map((_, index) => (
                <button
                  key={index}
                  className={clsx(
                    'homepage-testimonials__indicator',
                    index === currentTestimonial ? 'homepage-testimonials__indicator--active' : ''
                  )}
                  onClick={() => setCurrentTestimonial(index)}
                  aria-label={`Go to testimonial ${index + 1}`}
                />
              ))}
            </div>
          </div>
        ) : (
          // Single testimonial mode
          <div className="homepage-testimonials__single">
            {testimonials.length > 0 && <TestimonialCard {...testimonials[0]} />}
          </div>
        )}
      </div>
    </section>
  );
};

export default Testimonials;