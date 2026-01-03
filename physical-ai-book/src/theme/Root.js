import React from 'react';
import { ChatKitWrapper } from '../components/ChatKit/ChatKitWrapper';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatKitWrapper />
    </>
  );
}