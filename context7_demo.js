const { spawn } = require('child_process');
const readline = require('readline');

// This script demonstrates how to connect to and use Context7 MCP server
console.log("Starting Context7 MCP client demonstration...");

// Start the Context7 MCP server as a child process
const server = spawn('npx', ['-y', '@upstash/context7-mcp'], {
  stdio: ['pipe', 'pipe', 'pipe'],
  env: { ...process.env }
});

// Handle server errors and output
server.stderr.on('data', (data) => {
  const stderrOutput = data.toString();
  if (!stderrOutput.includes("WARNING: Using default CLIENT_IP_ENCRYPTION_KEY.")) {
    console.log(`Server output: ${stderrOutput.trim()}`);
  }
});

// Create readline interface to handle line-by-line reading
const rl = readline.createInterface({
  input: server.stdout,
  crlfDelay: Infinity
});

// Buffer to store incomplete JSON responses
let buffer = '';

// Handle incoming data from the server
rl.on('line', (chunk) => {
  buffer += chunk;

  // Try to parse the accumulated buffer
  try {
    const response = JSON.parse(buffer);
    console.log("Received from Context7 MCP server:", JSON.stringify(response, null, 2));
    buffer = ''; // Clear buffer after successful parse
  } catch (e) {
    // If it's not complete JSON, keep it in buffer
    // This might happen if we get partial JSON data
    console.log("Partial data received, waiting for complete message:", chunk);
  }
});

// Send an initial tools/list request to see what's available
setTimeout(() => {
  const toolsRequest = {
    header: {
      "mcp": "1.0",
      "request_id": "req-123",
      "method": "tools/list"
    },
    body: {
      "include": ["descriptions", "parameters"]
    }
  };

  const requestString = JSON.stringify(toolsRequest);
  console.log("Sending tools/list request to Context7:", requestString);
  server.stdin.write(requestString + '\n');
}, 2000);

// Send a test request to resolve a library ID after a delay
setTimeout(() => {
  const resolveRequest = {
    header: {
      "mcp": "1.0",
      "request_id": "req-456",
      "method": "tools/call",
      "id": "resolve-library-id"
    },
    body: {
      "arguments": {
        "libraryName": "react"
      }
    }
  };

  const requestString = JSON.stringify(resolveRequest);
  console.log("Sending resolve-library-id request to Context7:", requestString);
  server.stdin.write(requestString + '\n');
}, 4000);

// Clean up after the demonstration
setTimeout(() => {
  console.log("Demonstration complete. Stopping server...");
  server.kill();
  process.exit(0);
}, 8000);