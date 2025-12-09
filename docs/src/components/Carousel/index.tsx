import React, { useState, useEffect, useCallback } from 'react';
import type { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export interface CarouselProps {
    children: ReactNode[];
    autoPlay?: boolean;
    interval?: number;
    showDots?: boolean;
    showArrows?: boolean;
    className?: string;
}

export default function Carousel({
    children,
    autoPlay = true,
    interval = 5000,
    showDots = true,
    showArrows = true,
    className,
}: CarouselProps): ReactNode {
    const [currentIndex, setCurrentIndex] = useState(0);
    const [isHovered, setIsHovered] = useState(false);

    const goToSlide = useCallback((index: number) => {
        setCurrentIndex(index);
    }, []);

    const goToPrevious = useCallback(() => {
        setCurrentIndex((prevIndex) =>
            prevIndex === 0 ? children.length - 1 : prevIndex - 1
        );
    }, [children.length]);

    const goToNext = useCallback(() => {
        setCurrentIndex((prevIndex) =>
            prevIndex === children.length - 1 ? 0 : prevIndex + 1
        );
    }, [children.length]);

    useEffect(() => {
        if (!autoPlay || isHovered) return;

        const timer = setInterval(goToNext, interval);
        return () => clearInterval(timer);
    }, [autoPlay, interval, isHovered, goToNext]);

    return (
        <div
            className={clsx(styles.carousel, className)}
            onMouseEnter={() => setIsHovered(true)}
            onMouseLeave={() => setIsHovered(false)}
        >
            <div className={styles.carouselInner}>
                <div
                    className={styles.carouselTrack}
                    style={{
                        transform: `translateX(-${currentIndex * 100}%)`,
                    }}
                >
                    {children.map((child, index) => (
                        <div key={index} className={styles.carouselSlide}>
                            {child}
                        </div>
                    ))}
                </div>
            </div>

            {showArrows && (
                <>
                    <button
                        className={clsx(styles.carouselArrow, styles.carouselArrowLeft)}
                        onClick={goToPrevious}
                        aria-label="Previous slide"
                    >
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
                            <path d="M15 18l-6-6 6-6" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
                        </svg>
                    </button>
                    <button
                        className={clsx(styles.carouselArrow, styles.carouselArrowRight)}
                        onClick={goToNext}
                        aria-label="Next slide"
                    >
                        <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
                            <path d="M9 18l6-6-6-6" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" />
                        </svg>
                    </button>
                </>
            )}

            {showDots && (
                <div className={styles.carouselDots}>
                    {children.map((_, index) => (
                        <button
                            key={index}
                            className={clsx(
                                styles.carouselDot,
                                index === currentIndex && styles.carouselDotActive
                            )}
                            onClick={() => goToSlide(index)}
                            aria-label={`Go to slide ${index + 1}`}
                        />
                    ))}
                </div>
            )}
        </div>
    );
}
