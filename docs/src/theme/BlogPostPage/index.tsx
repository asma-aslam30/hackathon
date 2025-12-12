import React from 'react';
import clsx from 'clsx';
import { HtmlClassNameProvider, ThemeClassNames } from '@docusaurus/theme-common';
import { BlogPostProvider } from '@docusaurus/theme-common/internal';
import BlogLayout from '@theme/BlogLayout';
import BlogPostItem from '@theme/BlogPostItem';
import BlogPostPaginator from '@theme/BlogPostPaginator';
import BlogPostPageMetadata from '@theme/BlogPostPage/Metadata';
import TOC from '@theme/TOC';
import type { Props } from '@theme/BlogPostPage';
import Unlisted from '../Unlisted';
import styles from './styles.module.css';

function BlogPostPageContent({ children }: { children: React.ReactNode }): JSX.Element {
    const { metadata, toc } = children.type.contentType === 'blog-post' ? children.props : { metadata: {}, toc: undefined };
    const { frontMatter } = metadata;
    const {
        hide_table_of_contents: hideTableOfContents,
        toc_min_heading_level: tocMinHeadingLevel,
        toc_max_heading_level: tocMaxHeadingLevel,
    } = frontMatter;

    return (
        <BlogLayout
            sidebar={null}
            toc={
                !hideTableOfContents && toc && toc.length > 0 ? (
                    <div className={styles.tocWrapper}>
                        <div className={styles.tocSticky}>
                            <TOC
                                toc={toc}
                                minHeadingLevel={tocMinHeadingLevel}
                                maxHeadingLevel={tocMaxHeadingLevel}
                            />
                        </div>
                    </div>
                ) : undefined
            }>
            {metadata.unlisted && <Unlisted />}

            <div className={styles.blogPostContainer}>
                <article className={styles.blogPostArticle}>
                    <BlogPostItem>{children}</BlogPostItem>
                </article>

                <div className={styles.paginatorWrapper}>
                    <BlogPostPaginator />
                </div>
            </div>
        </BlogLayout>
    );
}

export default function BlogPostPage(props: Props): JSX.Element {
    const BlogPostContent = props.content;
    return (
        <BlogPostProvider content={props.content} isBlogPostPage>
            <HtmlClassNameProvider
                className={clsx(
                    ThemeClassNames.wrapper.blogPages,
                    ThemeClassNames.page.blogPostPage,
                )}>
                <BlogPostPageMetadata />
                <BlogPostPageContent>
                    <BlogPostContent />
                </BlogPostPageContent>
            </HtmlClassNameProvider>
        </BlogPostProvider>
    );
}
