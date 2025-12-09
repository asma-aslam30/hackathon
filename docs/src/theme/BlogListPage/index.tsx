import React from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {
    PageMetadata,
    HtmlClassNameProvider,
    ThemeClassNames,
} from '@docusaurus/theme-common';
import BlogLayout from '@theme/BlogLayout';
import BlogListPaginator from '@theme/BlogListPaginator';
import SearchMetadata from '@theme/SearchMetadata';
import type { Props } from '@theme/BlogListPage';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

function BlogListPageMetadata(props: Props): JSX.Element {
    const { metadata } = props;
    return (
        <>
            <PageMetadata title={metadata.blogTitle} description={metadata.blogDescription} />
            <SearchMetadata tag="blog_posts_list" />
        </>
    );
}

function BlogListPageContent(props: Props): JSX.Element {
    const { metadata, items } = props;

    return (
        <BlogLayout>
            {/* Hero Section */}
            <div className={styles.blogHero}>
                <div className={styles.heroContent}>
                    <Heading as="h1" className={styles.heroTitle}>
                        📝 Blog
                    </Heading>
                    <p className={styles.heroSubtitle}>
                        Insights, tutorials, and updates from our team
                    </p>
                </div>
            </div>

            {/* Blog Posts Grid */}
            <div className="container margin-vert--lg">
                <div className={styles.blogGrid}>
                    {items.map(({ content: BlogPostContent }, index) => {
                        const { metadata: postMetadata, frontMatter } = BlogPostContent;
                        const {
                            permalink,
                            title,
                            date,
                            formattedDate,
                            authors,
                            tags,
                            description,
                        } = postMetadata;

                        const isFeatured = index === 0;

                        return (
                            <article
                                key={permalink}
                                className={clsx(
                                    styles.blogCard,
                                    isFeatured && styles.featuredCard,
                                    'animate-fade-in-up',
                                    `stagger-${Math.min(index + 1, 5)}`
                                )}
                            >
                                <a href={permalink} className={styles.cardLink}>
                                    {/* Card Header with Image Placeholder */}
                                    <div className={styles.cardHeader}>
                                        <div className={styles.cardImage}>
                                            {isFeatured && (
                                                <div className={styles.featuredBadge}>
                                                    <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                                                        <path d="M12 2l3.09 6.26L22 9.27l-5 4.87 1.18 6.88L12 17.77l-6.18 3.25L7 14.14 2 9.27l6.91-1.01L12 2z" />
                                                    </svg>
                                                    Featured
                                                </div>
                                            )}
                                            <div className={styles.imagePlaceholder}>
                                                <svg width="64" height="64" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                                    <path d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5" />
                                                </svg>
                                            </div>
                                        </div>
                                    </div>

                                    {/* Card Content */}
                                    <div className={styles.cardContent}>
                                        <div className={styles.cardMeta}>
                                            <time dateTime={date} className={styles.cardDate}>
                                                {formattedDate}
                                            </time>
                                            {tags.length > 0 && (
                                                <div className={styles.cardTags}>
                                                    {tags.slice(0, 2).map((tag) => (
                                                        <span key={tag.permalink} className={styles.tag}>
                                                            {tag.label}
                                                        </span>
                                                    ))}
                                                </div>
                                            )}
                                        </div>

                                        <Heading as="h2" className={styles.cardTitle}>
                                            {title}
                                        </Heading>

                                        {description && (
                                            <p className={styles.cardDescription}>{description}</p>
                                        )}

                                        {authors.length > 0 && (
                                            <div className={styles.cardFooter}>
                                                <div className={styles.authors}>
                                                    {authors.map((author, idx) => (
                                                        <div key={idx} className={styles.author}>
                                                            {author.imageURL && (
                                                                <img
                                                                    src={author.imageURL}
                                                                    alt={author.name}
                                                                    className={styles.authorImage}
                                                                />
                                                            )}
                                                            <span className={styles.authorName}>{author.name}</span>
                                                        </div>
                                                    ))}
                                                </div>
                                                <div className={styles.readMore}>
                                                    Read more →
                                                </div>
                                            </div>
                                        )}
                                    </div>
                                </a>
                            </article>
                        );
                    })}
                </div>

                <BlogListPaginator metadata={metadata} />
            </div>
        </BlogLayout>
    );
}

export default function BlogListPage(props: Props): JSX.Element {
    return (
        <HtmlClassNameProvider
            className={clsx(
                ThemeClassNames.wrapper.blogPages,
                ThemeClassNames.page.blogListPage,
            )}>
            <BlogListPageMetadata {...props} />
            <BlogListPageContent {...props} />
        </HtmlClassNameProvider>
    );
}
