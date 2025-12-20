# GitHub MCP Server Integration

This project now includes a GitHub MCP (Model Context Protocol) server that enables Claude Code to interact with GitHub repositories for various operations.

## Setup Instructions

### 1. Prerequisites
- Node.js and npm must be installed on your system
- A GitHub Personal Access Token with appropriate permissions
- Required npm packages: @octokit/rest (install with `npm install @octokit/rest`)

### 2. Configure Your GitHub Token

Add your GitHub Personal Access Token to the `.env` file:

```bash
# Edit the .env file and replace the placeholder with your actual token
GITHUB_TOKEN=your_actual_github_personal_access_token_here
```

### 3. Environment Setup

Load your environment variables:

```bash
# Option 1: Export directly to your shell
export GITHUB_TOKEN="your_actual_token_here"

# Option 2: Load from .env file
source .env
```

### 4. Verify Installation

Run the test script to verify the GitHub MCP server is working:

```bash
python3 test_github_mcp.py
```

## GitHub MCP Capabilities

Once configured, the GitHub MCP server provides access to:

- **Repository Operations**: List repositories for authenticated user
- **Pull Request Management**: Create pull requests
- **Issue Tracking**: Create issues
- **GitHub API Access**: Access various GitHub API endpoints through MCP protocol

## Configuration Details

The MCP server configuration is located in `mcp-config.json`:

```json
{
  "mcpServers": {
    "context7": {
      "command": "npx",
      "args": ["-y", "@upstash/context7-mcp"],
      "env": {
        "CONTEXT7_API_KEY": "${CONTEXT7_API_KEY}"
      }
    },
    "github": {
      "command": "node",
      "args": ["github-mcp-server.js", "3001"],
      "env": {
        "GITHUB_TOKEN": "${GITHUB_TOKEN}"
      }
    }
  }
}
```

## Security Notes

- Never commit your actual GitHub token to version control
- The `.env` file is intended to be local and should not be committed
- Use fine-grained personal access tokens with minimal required permissions

## Troubleshooting

If you encounter issues:
1. Verify your GitHub token has the required permissions
2. Check that Node.js and npm are properly installed
3. Ensure environment variables are correctly set
4. Confirm the MCP server configuration is valid
5. Make sure the required npm packages are installed