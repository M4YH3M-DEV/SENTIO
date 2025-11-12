/**
 * React Hook for topic subscription with cleanup
 */

import { useEffect, useState } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';

export function useTopicSubscription<T>(
  client: ROSBridgeClient | null,
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

    let unsubscribe: (() => void) | null = null;

    client
      .subscribe<T>(
        {
          topic,
          messageType,
          throttleRate: 100,
          queueLength: 1,
        },
        (msg: T) => {
          setMessage(msg);
          setLastMessageTime(new Date());
          setMessageCount(prev => prev + 1);
        }
      )
      .then(unsubFn => {
        unsubscribe = unsubFn;
        setIsSubscribed(true);
      })
      .catch(error => {
        console.error(`Failed to subscribe to ${topic}:`, error);
        setIsSubscribed(false);
      });

    return () => {
      if (unsubscribe) {
        unsubscribe();
        setIsSubscribed(false);
      }
    };
  }, [client, topic, messageType, enabled]);

  return { message, lastMessageTime, messageCount, isSubscribed };
}
