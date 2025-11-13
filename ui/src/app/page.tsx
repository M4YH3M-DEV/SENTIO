'use client';

import React, { useState, useEffect } from 'react';
import DashboardLayout from './components/DashboardLayout';
import ConnectionStatus from './components/ConnectionStatus';
import SystemStatus from './components/SystemStatus';
import EmotionVisualizer from './components/EmotionVisualizer';
import TopicList from './components/TopicList';
import CameraFeed from './components/CameraFeed';
import ReasoningTrail from './components/ReasoningTrail';
import DemoController from './components/DemoController';
import ManualControl from './components/ManualControl';
import { useROSBridge } from '@/hooks/useROSBridge';

const DEFAULT_ROSBRIDGE_URL = 
  typeof window !== 'undefined' && process.env.NEXT_PUBLIC_ROSBRIDGE_URL
    ? process.env.NEXT_PUBLIC_ROSBRIDGE_URL
    : 'ws://localhost:9090';

export default function Home() {
  const { client, connected, status } = useROSBridge(DEFAULT_ROSBRIDGE_URL);
  const [activeTab, setActiveTab] = useState<'dashboard' | 'demo' | 'manual' | 'topics'>('dashboard');

  return (
    <div className="min-h-screen bg-sentio-primary">
      <DashboardLayout>
        {/* Header */}
        <div className="bg-sentio-secondary border-b border-sentio-accent/20 p-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <h1 className="text-3xl font-bold text-sentio-accent">SENTIO HUD</h1>
              <div className="text-xs text-gray-400">
                Version 0.1.0 | ROS 2 Rolling
              </div>
            </div>
            <ConnectionStatus 
              connected={connected} 
              url={DEFAULT_ROSBRIDGE_URL}
              status={status}
            />
          </div>
        </div>

        {/* Main Content */}
        <div className="p-6 space-y-6">
          {/* Status Overview */}
          <SystemStatus status={status} />

          {/* Tab Navigation */}
          <div className="flex gap-2 border-b border-sentio-accent/20">
            {[
              { id: 'dashboard' as const, label: 'Dashboard' },
              { id: 'demo' as const, label: 'Demos' },
              { id: 'manual' as const, label: 'Manual Control' },
              { id: 'topics' as const, label: 'Topics' },
            ].map(tab => (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id)}
                className={`px-4 py-2 text-sm font-medium transition-colors ${
                  activeTab === tab.id
                    ? 'border-b-2 border-sentio-accent text-sentio-accent'
                    : 'text-gray-400 hover:text-gray-300'
                }`}
              >
                {tab.label}
              </button>
            ))}
          </div>

          {/* Tab Content */}
          <div className="min-h-[600px]">
            {activeTab === 'dashboard' && (
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
                <EmotionVisualizer client={client} />
                <CameraFeed url={process.env.NEXT_PUBLIC_CAMERA_FEED_URL} />
                <ReasoningTrail client={client} />
              </div>
            )}

            {activeTab === 'demo' && (
              <DemoController client={client} />
            )}

            {activeTab === 'manual' && (
              <ManualControl client={client} />
            )}

            {activeTab === 'topics' && (
              <TopicList client={client} connected={connected} />
            )}
          </div>
        </div>
      </DashboardLayout>
    </div>
  );
}