'use client';

import React, { useState, useRef, useEffect } from 'react';

interface CameraFeedProps {
  url?: string;
}

export default function CameraFeed({ url }: CameraFeedProps) {
  const [isLoading, setIsLoading] = useState(true);
  const [hasError, setHasError] = useState(false);
  const imgRef = useRef<HTMLImageElement>(null);

  useEffect(() => {
    if (!url) {
      setHasError(true);
      return;
    }

    setIsLoading(true);
    setHasError(false);

    const img = imgRef.current;
    if (!img) return;

    const handleLoad = () => {
      setIsLoading(false);
      setHasError(false);
    };

    const handleError = () => {
      setIsLoading(false);
      setHasError(true);
    };

    img.addEventListener('load', handleLoad);
    img.addEventListener('error', handleError);
    img.src = url;

    return () => {
      img.removeEventListener('load', handleLoad);
      img.removeEventListener('error', handleError);
    };
  }, [url]);

  return (
    <div className="card space-y-4">
      <div className="card-header">
        <h2 className="text-lg font-bold">Camera Feed</h2>
        {hasError && (
          <div className="status-badge status-badge--disconnected">
            No Feed
          </div>
        )}
      </div>

      <div className="relative bg-sentio-primary rounded-lg overflow-hidden aspect-video border border-sentio-accent/20">
        {url ? (
          <>
            {isLoading && (
              <div className="absolute inset-0 bg-sentio-primary flex items-center justify-center">
                <div className="text-center">
                  <div className="animate-spin rounded-full h-12 w-12 border-4 border-sentio-accent border-t-transparent mx-auto mb-2"></div>
                  <div className="text-sm text-gray-400">Loading...</div>
                </div>
              </div>
            )}
            <img
              ref={imgRef}
              alt="Camera Feed"
              className="w-full h-full object-cover"
              onLoad={() => setIsLoading(false)}
            />
          </>
        ) : (
          <div className="absolute inset-0 flex items-center justify-center">
            <div className="text-center text-gray-400">
              <div className="text-3xl mb-2">📷</div>
              <div className="text-sm">Camera URL not configured</div>
            </div>
          </div>
        )}
      </div>

      {!hasError && url && (
        <div className="text-xs text-gray-400">
          MJPEG Stream: {url}
        </div>
      )}
    </div>
  );
}
