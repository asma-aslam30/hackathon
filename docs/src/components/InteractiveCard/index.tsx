import React, { useState, useRef, useEffect } from 'react';
import type { ReactNode, CSSProperties } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export type CardVariant = 'glass' | '3d' | 'gradient' | 'magnetic' | 'neon';

export interface InteractiveCardProps {
    variant?: CardVariant;
    title?: string;
    description?: ReactNode;
    icon?: ReactNode;
    accentColor?: 'cyan' | 'purple' | 'pink';
    className?: string;
    children?: ReactNode;
    onClick?: () => void;
}

export default function InteractiveCard({
    variant = 'glass',
    title,
    description,
    icon,
    accentColor = 'cyan',
    className,
    children,
    onClick,
}: InteractiveCardProps): ReactNode {
    const cardRef = useRef<HTMLDivElement>(null);
    const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
    const [isHovered, setIsHovered] = useState(false);

    // Magnetic hover effect
    useEffect(() => {
        if (variant !== 'magnetic' || !cardRef.current) return;

        const card = cardRef.current;
        const handleMouseMove = (e: MouseEvent) => {
            const rect = card.getBoundingClientRect();
            const x = e.clientX - rect.left - rect.width / 2;
            const y = e.clientY - rect.top - rect.height / 2;

            // Calculate distance from center
            const distance = Math.sqrt(x * x + y * y);
            const maxDistance = 100; // pixels

            if (distance < maxDistance) {
                const strength = 0.15; // magnetic strength
                setMousePosition({
                    x: x * strength,
                    y: y * strength,
                });
            } else {
                setMousePosition({ x: 0, y: 0 });
            }
        };

        const handleMouseLeave = () => {
            setMousePosition({ x: 0, y: 0 });
        };

        card.addEventListener('mousemove', handleMouseMove);
        card.addEventListener('mouseleave', handleMouseLeave);

        return () => {
            card.removeEventListener('mousemove', handleMouseMove);
            card.removeEventListener('mouseleave', handleMouseLeave);
        };
    }, [variant]);

    const cardStyle: CSSProperties = variant === 'magnetic' ? {
        transform: `translate(${mousePosition.x}px, ${mousePosition.y}px)`,
    } : undefined;

    const variantClass = {
        glass: styles.cardGlass,
        '3d': styles.card3d,
        gradient: styles.cardGradient,
        magnetic: styles.cardMagnetic,
        neon: styles.cardNeon,
    }[variant];

    const accentClass = {
        cyan: styles.accentCyan,
        purple: styles.accentPurple,
        pink: styles.accentPink,
    }[accentColor];

    return (
        <div
            ref={cardRef}
            className={clsx(
                styles.card,
                variantClass,
                accentClass,
                className,
                onClick && styles.clickable
            )}
            style={cardStyle}
            onClick={onClick}
            onMouseEnter={() => setIsHovered(true)}
            onMouseLeave={() => setIsHovered(false)}
            role={onClick ? 'button' : undefined}
            tabIndex={onClick ? 0 : undefined}
        >
            {variant === 'gradient' && (
                <div className={styles.gradientBorder} aria-hidden="true" />
            )}

            <div className={styles.cardContent}>
                {icon && (
                    <div className={clsx(styles.iconWrapper, isHovered && styles.iconHovered)}>
                        {icon}
                    </div>
                )}

                {title && (
                    <h3 className={styles.title}>{title}</h3>
                )}

                {description && (
                    <div className={styles.description}>{description}</div>
                )}

                {children}
            </div>

            {variant === 'neon' && (
                <div className={styles.neonGlow} aria-hidden="true" />
            )}
        </div>
    );
}
