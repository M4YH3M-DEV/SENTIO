#!/bin/bash
# Trigger demo using curl (requires rosbridge_server running)

ROSBRIDGE_URL="http://localhost:9090"
SEQUENCE_NAME="${1:-greet_sequence}"
VISITOR_NAME="${2:-Guest}"

echo "Triggering SENTIO demo: $SEQUENCE_NAME"
echo "Visitor: $VISITOR_NAME"
echo ""

# Call /demo/control service
curl -X POST "$ROSBRIDGE_URL/call_service" \
  -H "Content-Type: application/json" \
  -d "{
    \"service\": \"/demo/control\",
    \"args\": {
      \"command\": \"start\",
      \"sequence_name\": \"$SEQUENCE_NAME\",
      \"variables\": {
        \"visitor_name\": \"$VISITOR_NAME\"
      }
    }
  }"

echo ""
echo "Demo triggered!"
echo "Check /demo/status topic for progress"
