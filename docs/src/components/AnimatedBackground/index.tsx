import React from 'react';
import type { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export interface AnimatedBackgroundProps {
    variant?: 'mesh' | 'particles' | 'geometric';
    intensity?: 'low' | 'medium' | 'high';
    className?: string;
}

export default function AnimatedBackground({
    variant = 'mesh',
    intensity = 'medium',
    className,
}: AnimatedBackgroundProps): ReactNode {
    const variantClass = {
        mesh: styles.bgMesh,
        particles: styles.bgParticles,
        geometric: styles.bgGeometric,
    }[variant];

    const intensityClass = {
        low: styles.intensityLow,
        medium: styles.intensityMedium,
        high: styles.intensityHigh,
    }[intensity];

    return (
        <div className={clsx(styles.background, variantClass, intensityClass, className)} aria-hidden="true">
            {variant === 'mesh' && (
                <>
                    <div className={styles.gradientOrb1} />
                    <div className={styles.gradientOrb2} />
                    <div className={styles.gradientOrb3} />
                </>
            )}

            {variant === 'geometric' && (
                <>
                    <div className={styles.geometricShape1} />
                    <div className={styles.geometricShape2} />
                    <div className={styles.geometricShape3} />
                </>
            )}

            {variant === 'particles' && (
                <div className={styles.particleContainer}>
                    {Array.from({ length: 20 }).map((_, i) => (
                        <div key={i} className={styles.particle} style={{
                            '--delay': `${i * 0.5}s`,
                            '--duration': `${15 + i * 2}s`,
                            '--x': `${Math.random() * 100}%`,
                            '--y': `${Math.random() * 100}%`,
                        } as React.CSSProperties} />
                    ))}
                </div>
            )}
        </div>
    );
}
