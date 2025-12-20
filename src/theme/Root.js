/**
 * Root theme wrapper to inject ChatWidget and AuthProvider globally
 * Feature: 005-frontend-chat and auth system
 *
 * This component wraps the entire Docusaurus app to inject
 * the ChatWidget on all pages and provide auth context globally.
 */

import React from 'react';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
