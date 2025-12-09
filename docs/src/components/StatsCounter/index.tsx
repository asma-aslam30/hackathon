import React, { useState, useEffect, useRef } from 'react';
import type { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export interface StatsCounterProps {
    end: number;
    duration?: number;
    suffix?: string;
    prefix?: string;
    label: string;
    icon?: ReactNode;
    accentColor?: 'cyan' | 'purple' | 'pink' | 'orange' | 'green';
}

export default function StatsCounter({
    end,
    duration = 2000,
    suffix = '',
    prefix = '',
    label,
    icon,
    accentColor = 'cyan',
}: StatsCounterProps): ReactNode {
    const [count, setCount] = useState(0);
    const [isVisible, setIsVisible] = useState(false);
    const counterRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        const observer = new IntersectionObserver(
            ([entry]) => {
                if (entry.isIntersecting) {
                    setIsVisible(true);
                }
            },
            { threshold: 0.5 }
        );

        if (counterRef.current) {
            observer.observe(counterRef.current);
        }

        return () => observer.disconnect();
    }, []);

    useEffect(() => {
        if (!isVisible) return;

        const startTime = Date.now();
        const endTime = startTime + duration;

        const updateCount = () => {
            const now = Date.now();
            const progress = Math.min((now - startTime) / duration, 1);
            const easeOutQuart = 1 - Math.pow(1 - progress, 4);
            const currentCount = Math.floor(easeOutQuart * end);

            setCount(currentCount);

            if (now < endTime) {
                requestAnimationFrame(updateCount);
            } else {
                setCount(end);
            }
        };

        requestAnimationFrame(updateCount);
    }, [isVisible, end, duration]);

    const accentClass = {
        cyan: styles.accentCyan,
        purple: styles.accentPurple,
        pink: styles.accentPink,
        orange: styles.accentOrange,
        green: styles.accentGreen,
    }[accentColor];

    return (
        <div ref={counterRef} className={clsx(styles.statsCounter, accentClass)}>
            {icon && <div className={styles.statsIcon}>{icon}</div>}
            <div className={styles.statsNumber}>
                {prefix}{count.toLocaleString()}{suffix}
            </div>
            <div className={styles.statsLabel}>{label}</div>
        </div>
    );
}
