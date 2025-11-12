'use client';

import React, { useState } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';
import { useTopicSubscription } from '@/hooks/useTopicSubscription';
import { formatTimestamp, parseJSONSafely } from '@/lib/utils';

interface ReasoningTrailProps {
  client: ROSBridgeClient | null;
}

interface ReasoningEntry {
  timestamp: string;
  type: string;
  data: Record<string, unknown>;
  confidence?: number;
}

export default function ReasoningTrail({ client }: ReasoningTrailProps) {
  const { message: reasoningData } = useTopicSubscription<string>(
    client,
    '/aether/reasoning',
    'std_msgs/String'
  );

  const [entries, setEntries] = useState<ReasoningEntry[]>([]);
  const [expandedIndex, setExpandedIndex] = useState<number | null>(null);

  React.useEffect(() => {
    if (reasoningData) {
      const parsed = parseJSONSafely(reasoningData, null);
      if (parsed) {
        setEntries(prev => [parsed as ReasoningEntry, ...prev].slice(0, 20));
      }
    }
  }, [reasoningData]);

  return (
    <div className="card space-y-4 lg:col-span-2">
      <div className="card-header">
        <h2 className="text-lg font-bold">Reasoning Trail (AETHER)</h2>
        <div className="text-xs text-gray-400">{entries.length} entries</div>
      </div>

      <div className="space-y-2 max-h-96 overflow-y-auto">
        {entries.length === 0 ? (
          <div className="text-center text-gray-400 py-8">
            <div className="text-3xl mb-2">🔍</div>
            <div className="text-sm">No reasoning data available</div>
          </div>
        ) : (
          entries.map((entry, index) => (
            <div
              key={`${entry.timestamp}-${index}`}
              className="bg-sentio-primary border border-sentio-accent/20 rounded-lg overflow-hidden"
            >
              <button
                onClick={() => setExpandedIndex(expandedIndex === index ? null : index)}
                className="w-full px-4 py-3 text-left hover:bg-sentio-accent/5 transition-colors flex items-center justify-between"
              >
                <div className="flex-1">
                  <div className="text-xs text-gray-400">
                    {formatTimestamp(new Date(entry.timestamp))}
                  </div>
                  <div className="text-sm font-semibold text-sentio-accent mt-1">
                    {entry.type}
                  </div>
                  {entry.confidence !== undefined && (
                    <div className="text-xs text-gray-500 mt-1">
                      Confidence: {(entry.confidence * 100).toFixed(0)}%
                    </div>
                  )}
                </div>
                <div className={`transform transition-transform ${
                  expandedIndex === index ? 'rotate-180' : ''
                }`}>
                  ▼
                </div>
              </button>

              {expandedIndex === index && (
                <div className="px-4 py-3 bg-sentio-primary/50 border-t border-sentio-accent/10">
                  <pre className="text-xs font-mono text-gray-400 overflow-auto max-h-40">
                    {JSON.stringify(entry.data, null, 2)}
                  </pre>
                </div>
              )}
            </div>
          ))
        )}
      </div>
    </div>
  );
}
