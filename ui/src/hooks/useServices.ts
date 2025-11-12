/**
 * React Hook for ROS service calls
 */

import { useCallback, useState } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';

export function useService<Req, Res>(
  client: ROSBridgeClient | null,
  serviceName: string,
  serviceType: string
): {
  call: (request: Req) => Promise<Res>;
  loading: boolean;
  error: string | null;
  result: Res | null;
} {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [result, setResult] = useState<Res | null>(null);

  const call = useCallback(
    async (request: Req): Promise<Res> => {
      if (!client) {
        const err = 'ROSBridge client not connected';
        setError(err);
        throw new Error(err);
      }

      setLoading(true);
      setError(null);

      try {
        const response = await client.callService<Req, Res>(
          serviceName,
          serviceType,
          request
        );
        setResult(response);
        return response;
      } catch (err) {
        const errorMsg = err instanceof Error ? err.message : 'Unknown error';
        setError(errorMsg);
        throw err;
      } finally {
        setLoading(false);
      }
    },
    [client, serviceName, serviceType]
  );

  return { call, loading, error, result };
}
