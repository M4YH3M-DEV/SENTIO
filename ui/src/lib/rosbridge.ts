/**
 * ROSBridge WebSocket client wrapper
 * Provides a clean API for subscribing/publishing to ROS topics
 */

import ROSLIB from 'roslib';
import { AffectMessage, BehaviorCommand, TopicMessage } from './types';

export type MessageCallback<T> = (message: T) => void;
export type ConnectionCallback = (connected: boolean) => void;

interface SubscriptionConfig {
  topic: string;
  messageType: string;
  throttleRate?: number;
  queueLength?: number;
}

interface PublishConfig {
  topic: string;
  messageType: string;
}

export class ROSBridgeClient {
  private ros: ROSLIB.Ros;
  private url: string;
  private connected = false;
  private subscriptions: Map<string, ROSLIB.Topic> = new Map();
  private publishers: Map<string, ROSLIB.Topic> = new Map();
  private connectionCallbacks: Set<ConnectionCallback> = new Set();
  private messageCallbacks: Map<string, Set<MessageCallback<unknown>>> = new Map();
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 10;
  private reconnectDelay = 1000;

  constructor(url: string) {
    this.url = url;
    this.ros = new ROSLIB.Ros({ url });
    this.setupConnectionHandlers();
  }

  private setupConnectionHandlers(): void {
    this.ros.on('connection', () => {
      this.connected = true;
      this.reconnectAttempts = 0;
      console.log('Connected to ROSBridge');
      this.notifyConnectionChange(true);
    });

    this.ros.on('error', (error: unknown) => {
      console.error('ROSBridge error:', error);
      this.notifyConnectionChange(false);
    });

    this.ros.on('close', () => {
      this.connected = false;
      console.log('Disconnected from ROSBridge');
      this.notifyConnectionChange(false);
      this.attemptReconnect();
    });
  }

  private attemptReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = this.reconnectDelay * Math.pow(1.5, this.reconnectAttempts - 1);
      console.log(`Attempting to reconnect in ${Math.round(delay)}ms...`);
      
      setTimeout(() => {
        if (!this.connected) {
          this.ros.connect(this.url);
        }
      }, delay);
    }
  }

  private notifyConnectionChange(connected: boolean): void {
    this.connectionCallbacks.forEach(callback => callback(connected));
  }

  onConnectionChange(callback: ConnectionCallback): () => void {
    this.connectionCallbacks.add(callback);
    return () => this.connectionCallbacks.delete(callback);
  }

  isConnected(): boolean {
    return this.connected;
  }

  async subscribe<T>(
    config: SubscriptionConfig,
    callback: MessageCallback<T>
  ): Promise<() => void> {
    if (!this.subscriptions.has(config.topic)) {
      const topic = new ROSLIB.Topic({
        ros: this.ros,
        name: config.topic,
        messageType: config.messageType,
        throttle_rate: config.throttleRate || 0,
        queue_length: config.queueLength || 1,
      });

      this.subscriptions.set(config.topic, topic);

      if (!this.messageCallbacks.has(config.topic)) {
        this.messageCallbacks.set(config.topic, new Set());
      }

      topic.subscribe((message: unknown) => {
        const callbacks = this.messageCallbacks.get(config.topic);
        if (callbacks) {
          callbacks.forEach(cb => cb(message as T));
        }
      });
    }

    const callbacks = this.messageCallbacks.get(config.topic);
    if (callbacks) {
      callbacks.add(callback);
    }

    return () => {
      callbacks?.delete(callback);
      if (callbacks?.size === 0) {
        const topic = this.subscriptions.get(config.topic);
        if (topic) {
          topic.unsubscribe();
          this.subscriptions.delete(config.topic);
          this.messageCallbacks.delete(config.topic);
        }
      }
    };
  }

  async publish<T extends Record<string, unknown>>(
    config: PublishConfig,
    message: T
  ): Promise<void> {
    if (!this.publishers.has(config.topic)) {
      const topic = new ROSLIB.Topic({
        ros: this.ros,
        name: config.topic,
        messageType: config.messageType,
      });
      this.publishers.set(config.topic, topic);
    }

    const topic = this.publishers.get(config.topic);
    if (topic) {
      const rosMessage = new ROSLIB.Message(message);
      topic.publish(rosMessage);
    }
  }

  async callService<Req, Res>(
    service: string,
    messageType: string,
    request: Req
  ): Promise<Res> {
    return new Promise((resolve, reject) => {
      const rosService = new ROSLIB.Service({
        ros: this.ros,
        name: service,
        serviceType: messageType,
      });

      const rosRequest = new ROSLIB.ServiceRequest(request);

      rosService.callService(
        rosRequest,
        (response: unknown) => {
          resolve(response as Res);
        },
        (error: unknown) => {
          reject(error);
        }
      );
    });
  }

  disconnect(): void {
    this.subscriptions.forEach(topic => topic.unsubscribe());
    this.subscriptions.clear();
    this.publishers.clear();
    this.messageCallbacks.clear();
    this.ros.close();
  }
}

// Singleton instance
let clientInstance: ROSBridgeClient | null = null;

export function getROSBridgeClient(url: string): ROSBridgeClient {
  if (!clientInstance) {
    clientInstance = new ROSBridgeClient(url);
  }
  return clientInstance;
}

export function closeROSBridgeClient(): void {
  if (clientInstance) {
    clientInstance.disconnect();
    clientInstance = null;
  }
}
