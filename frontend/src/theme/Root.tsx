import React from 'react';
import ChatInterface from '../components/ChatInterface';

// This component wraps the entire Docusaurus app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <ChatInterface />
    </>
  );
}
