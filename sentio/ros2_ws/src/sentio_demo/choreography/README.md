# SENTIO Choreography Sequences

This directory contains YAML choreography sequences that define coordinated robot behaviors.

## Sequence Format

Each YAML file defines a sequence with the following structure:

name: "Sequence Name"
description: "Description of what the sequence does"

steps:

action: "action_type"

action-specific fields
delay_ms: 0 # Delay before executing (optional)
duration_ms: 1000 # Duration for this action (optional)

text

## Supported Actions

### Gesture Action
Triggers a physical gesture/pose:

action: "gesture"
gesture: "greet" # Gesture name
duration_ms: 1000 # Duration to hold/perform gesture
led: # Optional LED command
color: "cyan"
pattern: "pulse"
intensity: 0.8

text

Supported gestures: `idle`, `greet`, `wave`, `nod`, `shake_head`, `look_left`, `look_right`, `happy_bounce`, `listen`

### TTS Action
Triggers text-to-speech:

action: "tts"
text: "Hello world!" # Text to speak
voice: "sentio_friendly" # Optional voice profile
emotion: "happy" # Optional emotion

text

Supported emotions: `happy`, `sad`, `angry`, `neutral`, `concerned`, `supportive`

### Wait Action
Pause for a specified duration:

action: "wait"
duration_ms: 2000 # Wait duration

text

### LED Action
Direct LED control:

action: "led"
color: "blue" # Color name
pattern: "pulse" # Pattern: solid, pulse, blink
intensity: 0.8 # Brightness 0-1

text

## Variable Substitution

Sequences support variable substitution using `{variable_name}` syntax:

action: "tts"
text: "Hello, {visitor_name}!"

text

When running the demo, provide variables:
variables = {"visitor_name": "Alice"}
demo.start_demo("greet_sequence", variables)

text

## Timing

- `delay_ms`: Time to wait before executing this step
- `duration_ms`: How long the action lasts
- Total sequence duration is calculated from step timings

## Example: Create Your Own Sequence

Create a file `my_demo.yaml`:

name: "My Custom Demo"
description: "My own choreography"

steps:

action: "gesture"
gesture: "look_left"
delay_ms: 0
duration_ms: 500

action: "tts"
text: "Looking around..."
emotion: "neutral"
delay_ms: 200

action: "wait"
duration_ms: 1000

action: "gesture"
gesture: "happy_bounce"
delay_ms: 0
duration_ms: 2000
led:
color: "yellow"
pattern: "pulse"

action: "tts"
text: "I am happy!"
emotion: "happy"
delay_ms: 300

text

Then load it:
parser.load_sequence("choreography/my_demo.yaml")

text
undefined