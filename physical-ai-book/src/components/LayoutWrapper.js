import React from 'react';
import ReadingProgress from './ReadingProgress';

const LayoutWrapper = ({ children }) => {
  return (
    <>
      <ReadingProgress />
      {children}
    </>
  );
};

export default LayoutWrapper;