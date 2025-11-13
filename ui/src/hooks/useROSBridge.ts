'use client';

import { useState, useEffect, useCallback } from 'react';
import ROSLIB from 'roslib';

interface UseROSBridgeReturn {
  client: ROSLIB.Ros | null;
  connected: boolean;
  status: string;
}

export function useROSBridge(url: string): UseROSBridgeReturn {
  const [client, setClient] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);
  const [status, setStatus] = useState<string>('disconnected');

  useEffect(() => {
    // Create ROS connection
    const ros = new ROSLIB.Ros({
      url: url,
    });

    // Connection event handlers
    ros.on('connection', () => {
      console.log('✅ Connected to ROSBridge');
      setConnected(true);
      setStatus('connected');
    });

    ros.on('error', (error) => {
      console.error('❌ ROSBridge error:', error);
      setConnected(false);
      setStatus('error');
    });

    ros.on('close', () => {
      console.log('🔌 Disconnected from ROSBridge');
      setConnected(false);
      setStatus('disconnected');
    });

    setClient(ros);

    // Cleanup on unmount
    return () => {
      if (ros) {
        ros.close();
      }
    };
  }, [url]);

  return { client, connected, status };
}