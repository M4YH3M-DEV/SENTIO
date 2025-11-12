/**
 * TypeScript interfaces for ROS messages used in the UI
 */

export interface ROSMessage {
  topic: string;
  msg: unknown;
  secs: number;
  nsecs: number;
}

export interface AffectMessage {
  valence: number;  // -1.0 to +1.0
  arousal: number;  // 0.0 to 1.0
  label: number;
  label_text: string;
  confidence?: number;
}

export interface BehaviorCommand {
  gesture: string;
  led: {
    color: string;
    pattern: string;
    intensity: number;
  };
  tts: {
    text: string;
    voice?: string;
    emotion?: string;
  };
  valence?: number;
  arousal?: number;
  confidence?: number;
  duration_s?: number;
}

export interface MotionStatus {
  motion_state: string;
  commands_sent: number;
  errors: number;
  safety_status: {
    emergency_stop_active: boolean;
    is_falling: boolean;
    fall_reason?: string;
  };
  hardware_connected: boolean;
  simulate_mode: boolean;
}

export interface DemoStatus {
  state: string;
  sequence_name: string;
  current_step: number;
  total_steps: number;
  elapsed_ms: number;
  total_duration_ms: number;
  step_log: Array<{
    index: number;
    action: string;
    timestamp: string;
    status: string;
  }>;
}

export interface TTSStatus {
  status: string;
  path?: string;
  error?: string;
}

export interface ReasoningEntry {
  timestamp: string;
  type: string;
  data: Record<string, unknown>;
  confidence?: number;
  reasoning?: string;
  sequence?: number;
}

export interface TopicMessage {
  topic: string;
  messageType: string;
  lastMessage?: unknown;
  lastMessageTime?: Date;
  messageCount: number;
  subscribed: boolean;
}

export interface SystemStatus {
  connected: boolean;
  roslibConnected: boolean;
  connectionStatus: 'connected' | 'disconnected' | 'connecting' | 'error';
  errorMessage?: string;
  uptime: number;
  topicsSubscribed: number;
  topicsPublished: number;
}

export interface CameraStreamConfig {
  url: string;
  enabled: boolean;
  width: number;
  height: number;
  fps: number;
}

export interface UIConfig {
  rosbridgeUrl: string;
  cameraFeedUrl: string;
  refreshRateMs: number;
  enableCameraFeed: boolean;
  enableReasoningTrail: boolean;
  enableManualControl: boolean;
}
