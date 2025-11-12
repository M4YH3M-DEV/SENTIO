# SENTIO HUD - Web Dashboard

Real-time monitoring and control interface for SENTIO robot using Next.js and ROSBridge.

## Features

- **Live Emotion Visualization**: Real-time valence-arousal display
- **Camera Feed**: MJPEG streaming from robot cameras
- **Topic Monitoring**: Live list of ROS topics with message inspection
- **Demo Control**: Trigger and monitor choreography sequences
- **Manual Control**: Send gestures and TTS commands
- **Reasoning Trail**: View AETHER fusion and decision-making logs
- **System Status**: Monitor uptime and connection health

## Tech Stack

- **Frontend**: Next.js 14, React 18, TypeScript
- **Styling**: Tailwind CSS
- **ROS Bridge**: roslibjs
- **State Management**: React Hooks
- **Testing**: Jest + React Testing Library

## Setup

### Prerequisites

- Node.js 20+ (via nvm recommended)
- npm or yarn package manager
- Running ROSBridge server (ws://localhost:9090)

### Installation

Install nvm (if not already installed)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash

Install Node.js 20
nvm install 20
nvm use 20

Install dependencies
npm install

or
yarn install

text

### Configuration

Create `.env.local`:

cp .env.example .env.local

text

Edit `.env.local` to configure:

NEXT_PUBLIC_ROSBRIDGE_URL=ws://localhost:9090
NEXT_PUBLIC_CAMERA_FEED_URL=http://localhost:8080/stream
NEXT_PUBLIC_ENABLE_CAMERA_FEED=true
NEXT_PUBLIC_ENABLE_REASONING_TRAIL=true
NEXT_PUBLIC_ENABLE_MANUAL_CONTROL=true

text

## Running

### Development

npm run dev

or
yarn dev

text

Open [http://localhost:3000](http://localhost:3000) in your browser.

### Production Build

npm run build
npm start

or
yarn build
yarn start

text

### Type Checking

npm run type-check

text

### Testing

Run tests once
npm test

Watch mode
npm run test:watch

Coverage report
npm run test:coverage

text

### Formatting

npm run format

text

## Architecture

### Components

- **DashboardLayout**: Main layout wrapper
- **EmotionVisualizer**: Valence-arousal visualization
- **TopicList**: ROS topic inspector
- **CameraFeed**: Live camera streaming
- **ReasoningTrail**: AETHER decision logging
- **DemoController**: Sequence playback control
- **ManualControl**: Direct gesture/TTS input
- **SystemStatus**: Connection and uptime display

### Hooks

- **useROSBridge**: Initialize and manage ROSBridge connection
- **useTopicSubscription**: Subscribe to ROS topics
- **useServices**: Call ROS services

### Utilities

- **rosbridge.ts**: ROSBridge client wrapper
- **types.ts**: TypeScript interfaces for ROS messages
- **utils.ts**: Helper functions (formatting, emotion mapping, etc.)

## ROSBridge Integration

### Topics Subscribed

- `/affect` - Emotion state (valence/arousal)
- `/demo/status` - Demo playback status
- `/aether/reasoning` - Reasoning trails
- `/motion/status` - Motion control status
- `/tts_status` - TTS synthesis status

### Topics Published

- `/behavior_cmd` - Behavior commands (JSON)
- `/tts_text` - Text for synthesis
- `/demo/control` - Demo playback control

### Connection Management

- Auto-reconnect with exponential backoff
- Connection status display
- Graceful error handling

## Development

### Project Structure

src/
├── app/
│ ├── components/ # React components
│ ├── layout.tsx # Root layout
│ └── page.tsx # Home page
├── lib/
│ ├── rosbridge.ts # ROSBridge client
│ ├── types.ts # TypeScript types
│ └── utils.ts # Utilities
├── hooks/ # Custom React hooks
└── tests/ # Test files

text

### Adding Components

1. Create component in `src/app/components/`
2. Use TypeScript for type safety
3. Follow existing patterns for hooks and ROSBridge usage
4. Add tests in `src/__tests__/components/`

### Adding Topics

1. Add TypeScript interface in `src/lib/types.ts`
2. Use `useTopicSubscription` hook to subscribe
3. Update component with new data

## Troubleshooting

### ROSBridge Not Connecting

Verify ROSBridge is running
rostopic list

Check ROSBridge status
netstat -tlnp | grep 9090

Start ROSBridge if needed
ros2 launch rosbridge_server rosbridge_websocket.launch.xml

text

### Camera Feed Not Loading

- Verify camera stream URL in `.env.local`
- Check MJPEG server is running
- Verify CORS headers if streaming from different host

### TypeScript Errors

npm run type-check
npm run format

text

## Performance

### Optimization Tips

- Increase `throttle_rate` in `useTopicSubscription` to reduce update frequency
- Use `memo` for expensive components
- Lazy load non-critical components
- Monitor network usage in browser DevTools

## Deployment

### Vercel

Connect GitHub repo to Vercel
Or deploy directly:
npm i -g vercel
vercel

text

### Docker

FROM node:20-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build
EXPOSE 3000
CMD ["npm", "start"]

text

## License

Proprietary - DevSora Deep-Tech Research

## Support

For issues or questions, contact: dev@devsora.tech
Build and Verification Instructions
Step 1: Install Node.js 20
bash
# Install nvm (if needed)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc

# Install Node.js 20
nvm install 20
nvm use 20

# Verify
node --version  # Should be v20.x.x
npm --version   # Should be 9.x+
Step 2: Setup UI Directory
bash
mkdir -p ~/Documents/DEVSORA\ PROJECTS/SENTIO/ui
cd ~/Documents/DEVSORA\ PROJECTS/SENTIO/ui
Step 3: Create Configuration Files
Copy all files from Phase 9 into the ui/ directory with the exact structure shown.

Step 4: Install Dependencies
bash
cd ~/Documents/DEVSORA\ PROJECTS/SENTIO/ui
npm install
Step 5: Configure Environment
bash
cp .env.example .env.local

# Edit .env.local with your ROSBridge URL
# NEXT_PUBLIC_ROSBRIDGE_URL=ws://localhost:9090
Step 6: Run Tests
bash
npm test

# Expected output:
# PASS  src/__tests__/rosbridge.test.ts
# PASS  src/__tests__/components/EmotionVisualizer.test.tsx
# Test Suites: 2 passed, 2 total
Step 7: Type Check
bash
npm run type-check

# Should complete with no errors
Step 8: Start Development Server
bash
npm run dev

# Output:
# ▲ Next.js 14.0.0
# - Local: http://localhost:3000
Step 9: Verify in Browser
Open http://localhost:3000

Check connection status (should show "Disconnected" if ROSBridge not running)

Navigate tabs: Dashboard, Demos, Manual Control, Topics

Components should render without errors

Step 10: Start ROSBridge (in another terminal)
bash
# Terminal 1: Start ROSBridge
ros2 launch rosbridge_server rosbridge_websocket.launch.xml

# Terminal 2: Go back to ui and refresh browser
# Page should now show "Connected"
Step 11: Build for Production
bash
npm run build

# Output:
# ✓ Created .next build directory
# ✓ Compiled successfully
Step 12: Start Production Server
bash
npm start

# Open http://localhost:3000 in browser
# Should show production build