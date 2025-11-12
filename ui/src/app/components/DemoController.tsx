'use client';

import React, { useState, useEffect } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';
import { useTopicSubscription } from '@/hooks/useTopicSubscription';
import { formatDuration, parseJSONSafely } from '@/lib/utils';

interface DemoControllerProps {
  client: ROSBridgeClient | null;
}

interface DemoStatus {
  state: string;
  sequence_name: string;
  current_step: number;
  total_steps: number;
  elapsed_ms: number;
  total_duration_ms: number;
}

const DEMO_SEQUENCES = [
  { id: 'greet_sequence', name: 'Greeting', description: 'Simple greeting demo' },
  { id: 'showcase_full', name: 'Full Showcase', description: 'Complete capability demo' },
  { id: 'nod_sequence', name: 'Nod', description: 'Simple nod gesture' },
];

export default function DemoController({ client }: DemoControllerProps) {
  const { message: demoStatusMsg } = useTopicSubscription<string>(
    client,
    '/demo/status',
    'std_msgs/String'
  );

  const [demoStatus, setDemoStatus] = useState<DemoStatus | null>(null);
  const [selectedSequence, setSelectedSequence] = useState('greet_sequence');
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (demoStatusMsg) {
      const parsed = parseJSONSafely(demoStatusMsg, null);
      if (parsed) {
        setDemoStatus(parsed as DemoStatus);
      }
    }
  }, [demoStatusMsg]);

  const handleStartDemo = async () => {
    if (!client) return;

    setIsLoading(true);
    try {
      await client.publish(
        {
          topic: '/demo/control',
          messageType: 'std_msgs/String',
        },
        {
          data: JSON.stringify({
            command: 'start',
            sequence_name: selectedSequence,
          }),
        }
      );
    } catch (error) {
      console.error('Failed to start demo:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleStopDemo = async () => {
    if (!client) return;

    try {
      await client.publish(
        {
          topic: '/demo/control',
          messageType: 'std_msgs/String',
        },
        {
          data: JSON.stringify({ command: 'stop' }),
        }
      );
    } catch (error) {
      console.error('Failed to stop demo:', error);
    }
  };

  const handlePauseDemo = async () => {
    if (!client) return;

    try {
      await client.publish(
        {
          topic: '/demo/control',
          messageType: 'std_msgs/String',
        },
        {
          data: JSON.stringify({ command: 'pause' }),
        }
      );
    } catch (error) {
      console.error('Failed to pause demo:', error);
    }
  };

  const handleResumeDemo = async () => {
    if (!client) return;

    try {
      await client.publish(
        {
          topic: '/demo/control',
          messageType: 'std_msgs/String',
        },
        {
          data: JSON.stringify({ command: 'resume' }),
        }
      );
    } catch (error) {
      console.error('Failed to resume demo:', error);
    }
  };

  const isPlaying = demoStatus?.state === 'playing';
  const isPaused = demoStatus?.state === 'paused';
  const progress = demoStatus ? (demoStatus.current_step / demoStatus.total_steps) * 100 : 0;

  return (
    <div className="card space-y-6">
      <div className="card-header">
        <h2 className="text-xl font-bold">Demo Control</h2>
        {demoStatus && (
          <div className="text-xs text-gray-400">
            {demoStatus.state.toUpperCase()}
          </div>
        )}
      </div>

      {/* Sequence Selection */}
      <div className="space-y-3">
        <label className="block text-sm font-medium text-gray-300">
          Select Sequence
        </label>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
          {DEMO_SEQUENCES.map(seq => (
            <button
              key={seq.id}
              onClick={() => setSelectedSequence(seq.id)}
              disabled={isPlaying || isPaused}
              className={`p-4 rounded-lg border transition-all ${
                selectedSequence === seq.id
                  ? 'border-sentio-accent bg-sentio-accent/10'
                  : 'border-sentio-accent/20 hover:border-sentio-accent/40'
              } disabled:opacity-50 disabled:cursor-not-allowed`}
            >
              <div className="font-semibold text-left">{seq.name}</div>
              <div className="text-xs text-gray-400 text-left mt-1">
                {seq.description}
              </div>
            </button>
          ))}
        </div>
      </div>

      {/* Demo Status */}
      {demoStatus && (
        <div className="space-y-3 bg-sentio-primary p-4 rounded-lg border border-sentio-accent/20">
          <div className="flex items-center justify-between">
            <span className="text-sm text-gray-300">
              Step {demoStatus.current_step + 1} / {demoStatus.total_steps}
            </span>
            <span className="text-sm text-gray-400">
              {formatDuration(demoStatus.elapsed_ms)} / {formatDuration(demoStatus.total_duration_ms)}
            </span>
          </div>
          <div className="w-full bg-sentio-secondary rounded-full h-2 overflow-hidden border border-sentio-accent/20">
            <div
              className="h-full bg-gradient-to-r from-sentio-accent to-blue-500 transition-all duration-300"
              style={{ width: `${progress}%` }}
            ></div>
          </div>
        </div>
      )}

      {/* Controls */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
        <button
          onClick={handleStartDemo}
          disabled={isPlaying || isLoading}
          className="px-4 py-3 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white font-medium rounded-lg transition-colors"
        >
          {isLoading ? 'Starting...' : 'Start'}
        </button>

        <button
          onClick={handlePauseDemo}
          disabled={!isPlaying}
          className="px-4 py-3 bg-yellow-600 hover:bg-yellow-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white font-medium rounded-lg transition-colors"
        >
          Pause
        </button>

        <button
          onClick={handleResumeDemo}
          disabled={!isPaused}
          className="px-4 py-3 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white font-medium rounded-lg transition-colors"
        >
          Resume
        </button>

        <button
          onClick={handleStopDemo}
          disabled={!isPlaying && !isPaused}
          className="px-4 py-3 bg-red-600 hover:bg-red-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white font-medium rounded-lg transition-colors"
        >
          Stop
        </button>
      </div>
    </div>
  );
}
