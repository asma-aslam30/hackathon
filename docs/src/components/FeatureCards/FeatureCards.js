import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './featurecards.css';

const FeatureCard = ({ id, title, description, iconUrl, link }) => {
  return (
    <div className="homepage-feature-card">
      {iconUrl && (
        <div className="homepage-feature-card__icon">
          <img src={iconUrl} alt={title} className="homepage-feature-card__icon-img" />
        </div>
      )}
      <h3 className="homepage-feature-card__title">{title}</h3>
      <p className="homepage-feature-card__description">{description}</p>
      {link && (
        <Link to={link} className="homepage-feature-card__link">
          Learn more
        </Link>
      )}
    </div>
  );
};

const FeatureCards = ({ features = [] }) => {
  return (
    <section className="homepage-feature-cards">
      <div className="homepage-feature-cards__container">
        <div className="homepage-feature-cards__grid">
          {features.map((feature, index) => (
            <FeatureCard
              key={feature.id || index}
              id={feature.id}
              title={feature.title}
              description={feature.description}
              iconUrl={feature.iconUrl}
              link={feature.link}
            />
          ))}
        </div>
      </div>
    </section>
  );
};

export default FeatureCards;