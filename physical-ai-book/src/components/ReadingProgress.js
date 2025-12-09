import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const ReadingProgress = () => {
  const [progress, setProgress] = useState(0);
  const location = useLocation();

  useEffect(() => {
    const handleScroll = () => {
      const scrollTop = window.pageYOffset;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const scrolled = (scrollTop / docHeight) * 100;
      setProgress(Math.min(scrolled, 100));
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, [location.pathname]);

  return (
    <div className="progress-indicator" style={{ width: `${progress}%` }}></div>
  );
};

export default ReadingProgress;