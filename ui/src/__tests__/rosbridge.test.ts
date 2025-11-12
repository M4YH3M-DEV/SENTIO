import { ROSBridgeClient } from '@/lib/rosbridge';

describe('ROSBridgeClient', () => {
  let client: ROSBridgeClient;

  beforeEach(() => {
    client = new ROSBridgeClient('ws://localhost:9090');
  });

  afterEach(() => {
    client.disconnect();
  });

  test('should initialize with correct URL', () => {
    expect(client).toBeDefined();
  });

  test('should track connection state', (done) => {
    let callCount = 0;
    client.onConnectionChange((connected) => {
      callCount++;
      if (callCount === 1) {
        expect(typeof connected).toBe('boolean');
        done();
      }
    });
  });

  test('should handle subscription', async () => {
    const messages: unknown[] = [];
    
    const unsubscribe = await client.subscribe(
      {
        topic: '/test',
        messageType: 'std_msgs/String',
      },
      (msg) => {
        messages.push(msg);
      }
    );

    expect(typeof unsubscribe).toBe('function');
  });

  test('isConnected should return boolean', () => {
    expect(typeof client.isConnected()).toBe('boolean');
  });
});
