'use client';

import React, { useEffect, useState } from 'react';
import { SystemStatus } from '@/lib/types';
import { cn } from '@/lib/utils';

interface ConnectionStatusProps {
  connected: boolean;
  url: string;
  status: SystemStatus;
}

export default function ConnectionStatus({
  connected,
  url,
  status,
}: ConnectionStatusProps) {
  const [displayUrl, setDisplayUrl] = useState(url);

  useEffect(() => {
    setDisplayUrl(url);
  }, [url]);

  return (
    <div className="flex items-center gap-4">
      <div className="text-right">
        <div className="text-xs text-gray-400">ROSBridge</div>
        <div className="text-sm text-gray-200 font-mono">
          {displayUrl.replace('ws://', '').replace('wss://', '')}
        </div>
      </div>
      
      <div
        className={cn(
          'status-badge transition-all duration-300',
          connected ? 'status-badge--connected' : 'status-badge--disconnected'
        )}
      >
        <span
          className={cn(
            'inline-block w-2 h-2 rounded-full mr-2',
            connected ? 'bg-green-400' : 'bg-red-400',
            connected && 'animate-pulse'
          )}
        />
        {connected ? 'Connected' : 'Disconnected'}
      </div>
    </div>
  );
}
