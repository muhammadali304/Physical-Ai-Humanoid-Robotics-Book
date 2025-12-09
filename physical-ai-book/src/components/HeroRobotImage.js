import React from 'react';
import clsx from 'clsx';
import styles from './HeroRobotImage.module.css';

const HeroRobotImage = () => {
  return (
    <div className={clsx(styles.robotImageContainer, 'animate-float')}>
      <div className={styles.robotImage}>
        {/* SVG Robot Head - Sleek mechanical design */}
        <svg
          viewBox="0 0 200 200"
          className={styles.robotSvg}
          xmlns="http://www.w3.org/2000/svg"
        >
          {/* Robot head outline */}
          <path
            d="M 60 80 Q 50 60 70 40 Q 130 30 140 60 Q 150 80 140 100 Q 130 120 110 120 Q 90 120 80 110 Q 70 100 60 80 Z"
            fill="none"
            stroke="#2d3748"
            strokeWidth="3"
          />

          {/* Main head structure */}
          <path
            d="M 70 60 Q 80 45 100 45 Q 120 45 130 60 Q 135 75 135 90 Q 135 105 130 115 Q 120 125 100 125 Q 80 125 70 115 Q 65 105 65 90 Q 65 75 70 60 Z"
            fill="#2d3748"
            opacity="0.9"
          />

          {/* Eye detail */}
          <ellipse
            cx="100"
            cy="75"
            rx="8"
            ry="12"
            fill="#f6e05e"
            className={styles.eyeGlow}
          />
          <circle
            cx="100"
            cy="75"
            r="4"
            fill="#1a202c"
          />

          {/* Head panel details */}
          <path
            d="M 85 95 Q 100 90 115 95 Q 115 100 100 105 Q 85 100 85 95 Z"
            fill="#4a5568"
            opacity="0.7"
          />

          {/* Antenna */}
          <line
            x1="100"
            y1="45"
            x2="100"
            y2="30"
            stroke="#4a5568"
            strokeWidth="2"
          />
          <circle
            cx="100"
            cy="28"
            r="3"
            fill="#f6e05e"
            className={styles.antennaGlow}
          />

          {/* Side details */}
          <path
            d="M 65 80 Q 60 85 60 95 Q 60 105 65 110"
            fill="none"
            stroke="#4a5568"
            strokeWidth="2"
            opacity="0.6"
          />
          <path
            d="M 135 80 Q 140 85 140 95 Q 140 105 135 110"
            fill="none"
            stroke="#4a5568"
            strokeWidth="2"
            opacity="0.6"
          />

          {/* Mechanical lines */}
          <line
            x1="75"
            y1="65"
            x2="85"
            y2="70"
            stroke="#4a5568"
            strokeWidth="1"
            opacity="0.5"
          />
          <line
            x1="115"
            y1="70"
            x2="125"
            y2="65"
            stroke="#4a5568"
            strokeWidth="1"
            opacity="0.5"
          />
          <line
            x1="90"
            y1="110"
            x2="110"
            y2="110"
            stroke="#4a5568"
            strokeWidth="1"
            opacity="0.5"
          />
        </svg>

        {/* Glow effect */}
        <div className={styles.glowEffect}></div>
      </div>
    </div>
  );
};

export default HeroRobotImage;