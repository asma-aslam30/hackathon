import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './calltoaction.css';

const CallToAction = ({ actions = [] }) => {
  if (actions.length === 0) {
    return null;
  }

  return (
    <section className="homepage-cta">
      <div className="homepage-cta__container">
        <div className="homepage-cta__actions">
          {actions.map((action, index) => (
            <Link
              key={action.id || index}
              to={action.url}
              className={clsx(
                'homepage-cta__button',
                `homepage-cta__button--${action.type || 'primary'}`
              )}
              target={action.target || '_self'}
            >
              {action.text}
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
};

export default CallToAction;