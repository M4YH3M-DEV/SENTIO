'use client';

import React from 'react';

interface DashboardLayoutProps {
  children: React.ReactNode;
}

export default function DashboardLayout({ children }: DashboardLayoutProps) {
  return (
    <div className="flex flex-col h-screen bg-sentio-primary">
      <main className="flex-1 overflow-auto">
        {children}
      </main>
      <footer className="border-t border-sentio-accent/20 bg-sentio-secondary px-6 py-3 text-xs text-gray-400">
        <div className="flex items-center justify-between">
          <span>SENTIO Robot Control System</span>
          <span>© DevSora Deep-Tech Research</span>
        </div>
      </footer>
    </div>
  );
}
