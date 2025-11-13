'use client';

/**
 * React Hook for topic subscription with cleanup
 */

import { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

export function useTopicSubscription<T = any>(
  client: ROSLIB.Ros | null,
  topic: string,
  messageType: string,
  enabled = true
): {
  message: T | null;
  lastMessageTime: Date | null;
  messageCount: number;
  isSubscribed: boolean;
} {
  const [message, setMessage] = useState<T | null>(null);
  const [lastMessageTime, setLastMessageTime] = useState<Date | null>(null);
  const [messageCount, setMessageCount] = useState(0);
  const [isSubscribed, setIsSubscribed] = useState(false);

  useEffect(() => {
    if (!client || !enabled) return;

    // Create a topic listener
    const listener = new ROSLIB.Topic({
      ros: client,
      name: topic,
      messageType: messageType,
    });

    // Subscribe to the topic
    listener.subscribe((msg: ROSLIB.Message) => {
      setMessage(msg as T);
      setLastMessageTime(new Date());
      setMessageCount(prev => prev + 1);
    });

    setIsSubscribed(true);

    // Cleanup: unsubscribe on unmount
    return () => {
      listener.unsubscribe();
      setIsSubscribed(false);
    };
  }, [client, topic, messageType, enabled]);

  return { message, lastMessageTime, messageCount, isSubscribed };
}