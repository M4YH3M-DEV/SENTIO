'use client';

import React, { useState } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';

interface ManualControlProps {
  client: ROSBridgeClient | null;
}

const GESTURES = [
  'idle',
  'greet',
  'wave',
  'nod',
  'shake_head',
  'look_left',
  'look_right',
  'happy_bounce',
];

const LED_COLORS = [
  'white',
  'red',
  'green',
  'blue',
  'yellow',
  'cyan',
  'magenta',
  'orange',
];

export default function ManualControl({ client }: ManualControlProps) {
  const [selectedGesture, setSelectedGesture] = useState('idle');
  const [selectedLEDColor, setSelectedLEDColor] = useState('white');
  const [ttsText, setTtsText] = useState('Hello!');
  const [isLoading, setIsLoading] = useState(false);

  const handleSendGesture = async () => {
    if (!client) return;

    setIsLoading(true);
    try {
      const command = {
        gesture: selectedGesture,
        led: {
          color: selectedLEDColor,
          pattern: 'steady',
          intensity: 0.8,
        },
        tts: {
          text: '',
        },
      };

      await client.publish(
        {
          topic: '/behavior_cmd',
          messageType: 'std_msgs/String',
        },
        {
          data: JSON.stringify(command),
        }
      );
    } catch (error) {
      console.error('Failed to send gesture:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendTTS = async () => {
    if (!client || !ttsText.trim()) return;

    setIsLoading(true);
    try {
      await client.publish(
        {
          topic: '/tts_text',
          messageType: 'std_msgs/String',
        },
        {
          data: `text:${ttsText}`,
        }
      );
    } catch (error) {
      console.error('Failed to send TTS:', error);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="space-y-6">
      {/* Gesture Control */}
      <div className="card space-y-4">
        <div className="card-header">
          <h2 className="text-lg font-bold">Gesture Control</h2>
        </div>

        <div className="space-y-3">
          <label className="block text-sm font-medium text-gray-300">
            Select Gesture
          </label>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-2">
            {GESTURES.map(gesture => (
              <button
                key={gesture}
                onClick={() => setSelectedGesture(gesture)}
                className={`px-4 py-2 rounded-lg border text-sm font-medium transition-all ${
                  selectedGesture === gesture
                    ? 'border-sentio-accent bg-sentio-accent/10 text-sentio-accent'
                    : 'border-sentio-accent/20 text-gray-300 hover:border-sentio-accent/40'
                }`}
              >
                {gesture}
              </button>
            ))}
          </div>
        </div>

        <div className="space-y-3">
          <label className="block text-sm font-medium text-gray-300">
            LED Color
          </label>
          <div className="grid grid-cols-4 md:grid-cols-8 gap-2">
            {LED_COLORS.map(color => (
              <button
                key={color}
                onClick={() => setSelectedLEDColor(color)}
                className={`h-10 rounded-lg border-2 transition-all ${
                  selectedLEDColor === color
                    ? 'border-white scale-110'
                    : 'border-gray-600 hover:border-gray-400'
                }`}
                style={{
                  backgroundColor: color === 'white' ? '#ffffff' :
                                 color === 'red' ? '#ef4444' :
                                 color === 'green' ? '#10b981' :
                                 color === 'blue' ? '#3b82f6' :
                                 color === 'yellow' ? '#fbbf24' :
                                 color === 'cyan' ? '#06b6d4' :
                                 color === 'magenta' ? '#ec4899' :
                                 color === 'orange' ? '#f97316' : '#6b7280',
                }}
                title={color}
              />
            ))}
          </div>
        </div>

        <button
          onClick={handleSendGesture}
          disabled={isLoading}
          className="w-full px-4 py-3 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white font-medium rounded-lg transition-colors"
        >
          {isLoading ? 'Sending...' : 'Send Gesture'}
        </button>
      </div>

      {/* TTS Control */}
      <div className="card space-y-4">
        <div className="card-header">
          <h2 className="text-lg font-bold">Text-to-Speech</h2>
        </div>

        <div className="space-y-3">
          <label className="block text-sm font-medium text-gray-300">
            Text to Speak
          </label>
          <textarea
            value={ttsText}
            onChange={e => setTtsText(e.target.value)}
            className="w-full px-3 py-2 bg-sentio-primary border border-sentio-accent/20 rounded-lg text-white placeholder-gray-500 focus:outline-none focus:border-sentio-accent resize-none"
            rows={3}
            placeholder="Enter text to speak..."
          />
        </div>

        <button
          onClick={handleSendTTS}
          disabled={isLoading || !ttsText.trim()}
          className="w-full px-4 py-3 bg-purple-600 hover:bg-purple-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white font-medium rounded-lg transition-colors"
        >
          {isLoading ? 'Speaking...' : 'Speak'}
        </button>
      </div>
    </div>
  );
}
