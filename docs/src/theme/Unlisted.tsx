import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface UnlistedProps {
  className?: string;
}

export default function Unlisted({ className }: UnlistedProps): JSX.Element {
  return (
    <div className={clsx(styles.unlisted, className)}>
      <span className={styles.unlistedBadge}>Unlisted</span>
    </div>
  );
}