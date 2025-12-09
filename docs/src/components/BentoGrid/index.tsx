import React from 'react';
import type { ReactNode } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

export interface BentoGridProps {
    children: ReactNode;
    className?: string;
}

export interface BentoItemProps {
    children: ReactNode;
    size?: 'small' | 'medium' | 'large' | 'wide' | 'tall';
    className?: string;
}

export function BentoGrid({ children, className }: BentoGridProps): ReactNode {
    return (
        <div className={clsx(styles.bentoGrid, className)}>
            {children}
        </div>
    );
}

export function BentoItem({ children, size = 'medium', className }: BentoItemProps): ReactNode {
    const sizeClass = {
        small: styles.itemSmall,
        medium: styles.itemMedium,
        large: styles.itemLarge,
        wide: styles.itemWide,
        tall: styles.itemTall,
    }[size];

    return (
        <div className={clsx(styles.bentoItem, sizeClass, className)}>
            {children}
        </div>
    );
}

export default BentoGrid;
