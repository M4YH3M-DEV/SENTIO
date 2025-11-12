import React from 'react';
import { render, screen } from '@testing-library/react';
import EmotionVisualizer from '@/app/components/EmotionVisualizer';

describe('EmotionVisualizer', () => {
  test('renders emotion state card', () => {
    render(<EmotionVisualizer client={null} />);
    expect(screen.getByText('Emotion State')).toBeInTheDocument();
  });

  test('displays emotion label', () => {
    render(<EmotionVisualizer client={null} />);
    expect(screen.getByText('Unknown')).toBeInTheDocument();
  });

  test('shows valence and arousal metrics', () => {
    render(<EmotionVisualizer client={null} />);
    expect(screen.getByText('Valence')).toBeInTheDocument();
    expect(screen.getByText('Arousal')).toBeInTheDocument();
  });

  test('displays confidence percentage', () => {
    render(<EmotionVisualizer client={null} />);
    expect(screen.getByText(/Confidence/)).toBeInTheDocument();
  });

  test('renders LED preview section', () => {
    render(<EmotionVisualizer client={null} />);
    expect(screen.getByText('LED Preview')).toBeInTheDocument();
  });

  test('renders valence arousal plot', () => {
    render(<EmotionVisualizer client={null} />);
    expect(screen.getByText('Valence-Arousal Space')).toBeInTheDocument();
  });
});
