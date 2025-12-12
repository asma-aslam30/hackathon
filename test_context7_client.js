const { spawn } = require('child_process');
const readline = require('readline');

// Start the Context7 MCP server
const context7Server = spawn('npx', ['-y', '@upstash/context7-mcp'], {
  stdio: ['pipe', 'pipe', 'pipe'],
  env: { ...process.env }
});

// Set up readline interface to read from the server's stdout
const rl = readline.createInterface({
  input: context7Server.stdout,
  crlfDelay: Infinity
});

// Handle incoming messages from the server
rl.on('line', (line) => {
  console.log('Received from Context7 MCP:', line);
});

// Send an MCP handshake and tool discovery request
const handshakeMessage = {
  method: "mcp/begin-session",
  params: {}
};

context7Server.stdin.write(JSON.stringify(handshakeMessage) + '\n');

// After a brief delay, send a tool discovery request
setTimeout(() => {
  const discoveryRequest = {
    method: "tools/list",
    id: "1"
  };
  context7Server.stdin.write(JSON.stringify(discoveryRequest) + '\n');
}, 1000);

// Handle errors
context7Server.stderr.on('data', (data) => {
  console.error(`Context7 stderr: ${data}`);
});

// Clean up on exit
process.on('exit', () => {
  context7Server.kill();
});