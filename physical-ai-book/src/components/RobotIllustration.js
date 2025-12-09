import React from 'react';
import clsx from 'clsx';
import styles from './RobotIllustration.module.css';

const RobotIllustration = () => {
  return (
    <div className={clsx(styles.robotContainer, 'animate-float')}>
      <svg
        className={styles.robotSvg}
        viewBox="0 0 200 200"
        xmlns="http://www.w3.org/2000/svg"
      >
        {/* Robot head with more details */}
        <circle cx="100" cy="60" r="22" fill="none" stroke="var(--ifm-color-primary)" strokeWidth="2" />

        {/* Robot face details */}
        <circle cx="90" cy="55" r="2" fill="var(--ifm-color-primary)" />
        <circle cx="110" cy="55" r="2" fill="var(--ifm-color-primary)" />
        <path d="M 85 70 Q 100 75 115 70" stroke="var(--ifm-color-primary)" strokeWidth="1" fill="none" />

        {/* Robot neck */}
        <rect x="95" y="82" width="10" height="8" fill="none" stroke="var(--ifm-color-primary)" strokeWidth="2" />

        {/* Robot body - more humanoid shape */}
        <path d="M 80 90 L 80 130 Q 80 140 90 140 L 110 140 Q 120 140 120 130 L 120 90 Z"
              fill="none" stroke="var(--ifm-color-primary)" strokeWidth="2" />

        {/* Robot arms with joints */}
        <line x1="80" y1="100" x2="60" y2="100" stroke="var(--ifm-color-primary)" strokeWidth="2" />
        <circle cx="60" cy="100" r="3" fill="var(--ifm-color-primary)" />
        <line x1="60" y1="100" x2="45" y2="110" stroke="var(--ifm-color-primary)" strokeWidth="2" />

        <line x1="120" y1="100" x2="140" y2="100" stroke="var(--ifm-color-primary)" strokeWidth="2" />
        <circle cx="140" cy="100" r="3" fill="var(--ifm-color-primary)" />
        <line x1="140" y1="100" x2="155" y2="110" stroke="var(--ifm-color-primary)" strokeWidth="2" />

        {/* Robot legs */}
        <line x1="90" y1="140" x2="90" y2="160" stroke="var(--ifm-color-primary)" strokeWidth="2" />
        <line x1="110" y1="140" x2="110" y2="160" stroke="var(--ifm-color-primary)" strokeWidth="2" />
        <circle cx="90" cy="160" r="3" fill="var(--ifm-color-primary)" />
        <circle cx="110" cy="160" r="3" fill="var(--ifm-color-primary)" />

        {/* Decorative elements */}
        <circle cx="100" cy="75" r="1.5" fill="var(--ifm-color-tech-purple)" />
        <circle cx="85" cy="95" r="1" fill="var(--ifm-color-tech-teal)" />
        <circle cx="115" cy="95" r="1" fill="var(--ifm-color-tech-teal)" />

        {/* Chest panel */}
        <rect x="92" y="115" width="16" height="10" rx="2" fill="none" stroke="var(--ifm-color-tech-blue)" strokeWidth="1" />
      </svg>

      {/* Animated particles around robot */}
      <div className={styles.particle} style={{top: '15%', left: '20%'}}></div>
      <div className={styles.particle} style={{top: '70%', left: '85%'}}></div>
      <div className={styles.particle} style={{top: '25%', left: '80%'}}></div>
      <div className={styles.particle} style={{top: '65%', left: '15%'}}></div>
    </div>
  );
};

export default RobotIllustration;