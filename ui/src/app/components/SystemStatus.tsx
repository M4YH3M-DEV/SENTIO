'use client';

import React from 'react';
import { SystemStatus as SystemStatusType } from '@/lib/types';
import { formatDuration } from '@/lib/utils';

interface SystemStatusProps {
  status: SystemStatusType;
}

export default function SystemStatus({ status }: SystemStatusProps) {
  return (
    <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
      <div className="card">
        <div className="text-xs text-gray-400 uppercase tracking-wider">Status</div>
        <div className={`text-lg font-bold mt-1 ${
          status.connected ? 'text-green-400' : 'text-red-400'
        }`}>
          {status.connectionStatus === 'connected' ? 'Online' : 'Offline'}
        </div>
      </div>

      <div className="card">
        <div className="text-xs text-gray-400 uppercase tracking-wider">Uptime</div>
        <div className="text-lg font-bold text-sentio-accent mt-1">
          {formatDuration(status.uptime)}
        </div>
      </div>

      <div className="card">
        <div className="text-xs text-gray-400 uppercase tracking-wider">Topics Subscribed</div>
        <div className="text-lg font-bold text-blue-400 mt-1">
          {status.topicsSubscribed}
        </div>
      </div>

      <div className="card">
        <div className="text-xs text-gray-400 uppercase tracking-wider">Topics Publishing</div>
        <div className="text-lg font-bold text-purple-400 mt-1">
          {status.topicsPublished}
        </div>
      </div>
    </div>
  );
}
