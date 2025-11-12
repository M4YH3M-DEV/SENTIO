/**
 * Utility functions for the UI
 */

import { type ClassValue, clsx } from 'clsx';
import { twMerge } from 'tailwind-merge';

export function cn(...inputs: ClassValue[]): string {
  return twMerge(clsx(inputs));
}

export function formatTimestamp(date: Date): string {
  return new Intl.DateTimeFormat('en-US', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    fractionalSecondDigits: 3,
  }).format(date);
}

export function formatDuration(ms: number): string {
  const totalSeconds = Math.floor(ms / 1000);
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  const milliseconds = ms % 1000;

  if (minutes > 0) {
    return `${minutes}m ${seconds}s`;
  }
  return `${seconds}s ${milliseconds}ms`;
}

export function emotionToColor(emotion: string): string {
  const emotionMap: Record<string, string> = {
    happy: '#fbbf24',
    sad: '#3b82f6',
    angry: '#ef4444',
    fearful: '#f97316',
    disgusted: '#10b981',
    surprised: '#a78bfa',
    neutral: '#6b7280',
  };
  return emotionMap[emotion.toLowerCase()] || '#6b7280';
}

export function emotionToLabel(valence: number, arousal: number): string {
  if (valence > 0.5 && arousal > 0.5) return 'Happy';
  if (valence < -0.5 && arousal > 0.5) return 'Angry';
  if (valence < -0.5 && arousal < 0.5) return 'Sad';
  if (valence > 0.5 && arousal < 0.5) return 'Content';
  if (arousal > 0.7) return 'Excited';
  if (arousal < 0.3) return 'Calm';
  return 'Neutral';
}

export function clampValue(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

export function degreesToRadians(degrees: number): number {
  return (degrees * Math.PI) / 180;
}

export function radiansToDegrees(radians: number): number {
  return (radians * 180) / Math.PI;
}

export function parseJSONSafely<T>(json: string, fallback: T): T {
  try {
    return JSON.parse(json);
  } catch {
    return fallback;
  }
}

export function getEmotionIntensity(arousal: number): string {
  if (arousal < 0.33) return 'Subtle';
  if (arousal < 0.66) return 'Moderate';
  return 'Intense';
}
