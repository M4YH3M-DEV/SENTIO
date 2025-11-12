'use client';

import React, { useState, useMemo } from 'react';
import { ROSBridgeClient } from '@/lib/rosbridge';
import { useTopicSubscription } from '@/hooks/useTopicSubscription';
import { AffectMessage } from '@/lib/types';
import { emotionToColor, emotionToLabel, getEmotionIntensity } from '@/lib/utils';

interface EmotionVisualizerProps {
  client: ROSBridgeClient | null;
}

export default function EmotionVisualizer({ client }: EmotionVisualizerProps) {
  const { message: affectData } = useTopicSubscription<AffectMessage>(
    client,
    '/affect',
    'sentio_msgs/Affect'
  );

  const emotionData = useMemo(() => {
    if (!affectData) {
      return {
        emotion: 'Unknown',
        valence: 0,
        arousal: 0.5,
        confidence: 0,
        intensity: 'Unknown',
        color: '#6b7280',
      };
    }

    const emotion = emotionToLabel(affectData.valence, affectData.arousal);
    const intensity = getEmotionIntensity(affectData.arousal);
    const color = emotionToColor(emotion);

    return {
      emotion,
      valence: affectData.valence,
      arousal: affectData.arousal,
      confidence: affectData.confidence || 0,
      intensity,
      color,
    };
  }, [affectData]);

  return (
    <div className="card space-y-6">
      <div className="card-header">
        <h2 className="text-xl font-bold">Emotion State</h2>
        <div className="text-xs text-gray-400">
          Confidence: {(emotionData.confidence * 100).toFixed(0)}%
        </div>
      </div>

      {/* Large Emotion Visualization */}
      <div className="flex items-center justify-center">
        <div
          className="w-48 h-48 rounded-full shadow-2xl transition-all duration-300 flex items-center justify-center border-4 border-sentio-accent/30"
          style={{
            background: `radial-gradient(circle, ${emotionData.color}40 0%, ${emotionData.color}20 100%)`,
            opacity: 0.5 + emotionData.confidence * 0.5,
          }}
        >
          <div className="text-center">
            <div className="text-4xl font-bold text-white">
              {emotionData.emotion}
            </div>
            <div className="text-sm text-gray-300 mt-2">
              {emotionData.intensity}
            </div>
          </div>
        </div>
      </div>

      {/* Valence-Arousal Plot */}
      <div className="space-y-2">
        <h3 className="text-sm font-semibold text-gray-300">Valence-Arousal Space</h3>
        <div className="bg-sentio-primary rounded-lg p-4 aspect-square relative border border-sentio-accent/20">
          {/* Grid */}
          <div className="absolute inset-0 grid grid-cols-2 border border-sentio-accent/10 rounded-lg">
            <div className="border-r border-b border-sentio-accent/10"></div>
            <div className="border-b border-sentio-accent/10"></div>
          </div>

          {/* Axes Labels */}
          <div className="absolute top-1 left-1 text-xs text-gray-500">POSITIVE</div>
          <div className="absolute bottom-1 left-1 text-xs text-gray-500">NEGATIVE</div>
          <div className="absolute top-1 right-1 text-xs text-gray-500">EXCITED</div>
          <div className="absolute bottom-1 right-1 text-xs text-gray-500">CALM</div>

          {/* Point */}
          <div
            className="absolute w-4 h-4 rounded-full border-2 border-current transition-all duration-300 -translate-x-1/2 -translate-y-1/2"
            style={{
              left: `${((emotionData.valence + 1) / 2) * 100}%`,
              top: `${(1 - emotionData.arousal) * 100}%`,
              color: emotionData.color,
            }}
          >
            <div
              className="absolute inset-1 rounded-full opacity-30"
              style={{ backgroundColor: emotionData.color }}
            ></div>
          </div>
        </div>
      </div>

      {/* LED Preview */}
      <div className="space-y-2">
        <h3 className="text-sm font-semibold text-gray-300">LED Preview</h3>
        <div className="flex gap-2">
          <div
            className="flex-1 h-12 rounded-lg border border-sentio-accent/20 shadow-lg"
            style={{
              backgroundColor: emotionData.color,
              opacity: 0.5 + emotionData.confidence * 0.5,
            }}
          ></div>
          <div className="flex-1 h-12 rounded-lg border border-sentio-accent/20 bg-sentio-primary flex items-center justify-center text-xs text-gray-400">
            {emotionData.emotion}
          </div>
        </div>
      </div>

      {/* Metrics */}
      <div className="grid grid-cols-2 gap-3 pt-4 border-t border-sentio-accent/20">
        <div>
          <div className="text-xs text-gray-400">Valence</div>
          <div className="text-lg font-mono text-sentio-accent">
            {emotionData.valence.toFixed(2)}
          </div>
        </div>
        <div>
          <div className="text-xs text-gray-400">Arousal</div>
          <div className="text-lg font-mono text-sentio-accent">
            {emotionData.arousal.toFixed(2)}
          </div>
        </div>
      </div>
    </div>
  );
}
