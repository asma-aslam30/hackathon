import React from 'react';
import type { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export interface ImageShowcaseProps {
    images: {
        src: string;
        alt: string;
        title?: string;
        description?: string;
    }[];
    columns?: 2 | 3 | 4;
    className?: string;
}

export default function ImageShowcase({
    images,
    columns = 3,
    className,
}: ImageShowcaseProps): ReactNode {
    const columnClass = {
        2: styles.columns2,
        3: styles.columns3,
        4: styles.columns4,
    }[columns];

    return (
        <div className={clsx(styles.showcase, columnClass, className)}>
            {images.map((image, idx) => (
                <div key={idx} className={clsx(styles.showcaseItem, `stagger-${idx + 1}`)}>
                    <div className={styles.imageWrapper}>
                        <img
                            src={image.src}
                            alt={image.alt}
                            className={styles.image}
                            loading="lazy"
                        />
                        <div className={styles.imageOverlay}>
                            {image.title && (
                                <h4 className={styles.imageTitle}>{image.title}</h4>
                            )}
                            {image.description && (
                                <p className={styles.imageDescription}>{image.description}</p>
                            )}
                        </div>
                    </div>
                </div>
            ))}
        </div>
    );
}
