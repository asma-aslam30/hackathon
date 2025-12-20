import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './header.css';

const Header = ({ title, logoUrl, navigationItems = [], ctaButton }) => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className="homepage-header">
      <nav className="homepage-header__nav">
        <div className="homepage-header__logo">
          {logoUrl ? (
            <img src={logoUrl} alt={title || siteConfig.title} className="homepage-header__logo-img" />
          ) : null}
          <Link to="/" className="homepage-header__title">
            {title || siteConfig.title}
          </Link>
        </div>

        <div className="homepage-header__menu">
          <ul className="homepage-header__nav-list">
            {navigationItems.map((item, index) => (
              <li key={index} className="homepage-header__nav-item">
                <Link
                  to={item.url}
                  className={clsx(
                    'homepage-header__nav-link',
                    `homepage-header__nav-link--${item.type}`
                  )}
                >
                  {item.label}
                </Link>
              </li>
            ))}
          </ul>
        </div>

        {ctaButton && (
          <div className="homepage-header__cta">
            <Link
              to={ctaButton.url}
              className={clsx(
                'homepage-header__cta-button',
                `homepage-header__cta-button--${ctaButton.type || 'primary'}`
              )}
              target={ctaButton.target || '_self'}
            >
              {ctaButton.text}
            </Link>
          </div>
        )}
      </nav>
    </header>
  );
};

export default Header;