'use client';

import React, { useState, useEffect } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';
import { formatTimestamp, parseJSONSafely } from '@/lib/utils';

interface TopicListProps {
  client: ROSBridgeClient | null;
}

interface TopicInfo {
  name: string;
  type: string;
  message?: unknown;
  lastMessageTime?: Date;
  subscribers: number;
  publishers: number;
}

export default function TopicList({ client }: TopicListProps) {
  const [topics, setTopics] = useState<Map<string, TopicInfo>>(new Map());
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedTopic, setSelectedTopic] = useState<string | null>(null);

  useEffect(() => {
    if (!client?.isConnected()) return;

    // In a real implementation, would fetch topics from ROS
    const commonTopics = [
      { name: '/affect', type: 'sentio_msgs/Affect' },
      { name: '/tts_text', type: 'std_msgs/String' },
      { name: '/behavior_cmd', type: 'std_msgs/String' },
      { name: '/motion/status', type: 'std_msgs/String' },
      { name: '/demo/status', type: 'std_msgs/String' },
      { name: '/camera/image_raw', type: 'sensor_msgs/Image' },
      { name: '/imu/data', type: 'sensor_msgs/Imu' },
    ];

    const topicMap = new Map<string, TopicInfo>();
    commonTopics.forEach(topic => {
      topicMap.set(topic.name, {
        name: topic.name,
        type: topic.type,
        subscribers: Math.floor(Math.random() * 5),
        publishers: Math.floor(Math.random() * 3),
      });
    });

    setTopics(topicMap);
  }, [client]);

  const filteredTopics = Array.from(topics.values()).filter(topic =>
    topic.name.toLowerCase().includes(searchQuery.toLowerCase())
  );

  const selectedTopicInfo = selectedTopic
    ? topics.get(selectedTopic)
    : null;

  return (
    <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
      {/* Topic List */}
      <div className="lg:col-span-1 card space-y-4">
        <div className="card-header">
          <h2 className="text-lg font-bold">Topics</h2>
        </div>

        <input
          type="text"
          placeholder="Search topics..."
          value={searchQuery}
          onChange={e => setSearchQuery(e.target.value)}
          className="w-full px-3 py-2 bg-sentio-primary border border-sentio-accent/20 rounded-lg text-sm text-white placeholder-gray-500 focus:outline-none focus:border-sentio-accent"
        />

        <div className="space-y-2 max-h-96 overflow-y-auto">
          {filteredTopics.map(topic => (
            <button
              key={topic.name}
              onClick={() => setSelectedTopic(topic.name)}
              className={`w-full text-left px-3 py-2 rounded-lg text-sm transition-colors ${
                selectedTopic === topic.name
                  ? 'bg-sentio-accent/20 border border-sentio-accent text-sentio-accent'
                  : 'hover:bg-sentio-accent/10 border border-sentio-accent/10'
              }`}
            >
              <div className="font-mono text-xs">{topic.name}</div>
              <div className="text-xs text-gray-400 mt-1">{topic.type}</div>
            </button>
          ))}
        </div>
      </div>

      {/* Topic Details */}
      {selectedTopicInfo && (
        <div className="lg:col-span-2 card space-y-4">
          <div className="card-header">
            <h2 className="text-lg font-bold">{selectedTopicInfo.name}</h2>
          </div>

          <div className="grid grid-cols-2 gap-4">
            <div>
              <div className="text-xs text-gray-400">Type</div>
              <div className="text-sm font-mono mt-1 text-sentio-accent">
                {selectedTopicInfo.type}
              </div>
            </div>
            <div>
              <div className="text-xs text-gray-400">Publishers</div>
              <div className="text-sm font-bold mt-1 text-green-400">
                {selectedTopicInfo.publishers}
              </div>
            </div>
            <div>
              <div className="text-xs text-gray-400">Subscribers</div>
              <div className="text-sm font-bold mt-1 text-blue-400">
                {selectedTopicInfo.subscribers}
              </div>
            </div>
            <div>
              <div className="text-xs text-gray-400">Last Update</div>
              <div className="text-sm mt-1 text-gray-300">
                {selectedTopicInfo.lastMessageTime
                  ? formatTimestamp(selectedTopicInfo.lastMessageTime)
                  : 'N/A'}
              </div>
            </div>
          </div>

          {selectedTopicInfo.message && (
            <div>
              <div className="text-xs text-gray-400 mb-2">Message</div>
              <pre className="bg-sentio-primary p-3 rounded-lg text-xs font-mono text-gray-300 overflow-auto max-h-48 border border-sentio-accent/20">
                {JSON.stringify(selectedTopicInfo.message, null, 2)}
              </pre>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
