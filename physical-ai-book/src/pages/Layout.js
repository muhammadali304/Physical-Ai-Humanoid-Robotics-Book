/**
 * Custom Layout component to integrate ChatKit at the layout level
 */
import React from 'react';
import { ChatKitWrapper } from '../components/ChatKit/ChatKitWrapper';
import Layout from '@theme-original/Layout';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
        <ChatKitWrapper />
      </Layout>
    </>
  );
}