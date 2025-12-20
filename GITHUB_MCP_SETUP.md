# GitHub MCP Server Setup

This document explains how to set up and use the GitHub MCP server with Claude Code.

## Prerequisites

1. You need a GitHub Personal Access Token with appropriate permissions.
2. Node.js and npm must be installed on your system.
3. Required npm packages: @octokit/rest

## Creating a GitHub Personal Access Token

1. Go to GitHub Settings > Developer settings > Personal access tokens > Tokens (classic)
2. Click "Generate new token"
3. Select the scopes you need (at minimum):
   - `repo` - to work with repositories
   - `read:org` - to read organization info
   - `read:user` - to read user info
   - `gist` - to work with gists
   - `notifications` - to read notifications
4. Copy the generated token

## Configuration

The MCP configuration is already set up in `mcp-config.json`. You just need to provide your token:

```bash
export GITHUB_TOKEN="your_actual_token_here"
```

Or update the .env file with your token:

```bash
GITHUB_TOKEN=your_actual_token_here
```

## Available GitHub MCP Capabilities

Once configured, the GitHub MCP server provides access to:

1. **Repository Operations**:
   - List repositories for authenticated user
   - Access repository information

2. **Pull Request Management**:
   - Create pull requests

3. **Issue Tracking**:
   - Create issues

4. **Additional GitHub Features**:
   - Repository analysis
   - GitHub API access

## Usage Examples

After setting up your token, you can use GitHub MCP functions like:

- Listing your repositories
- Creating new issues
- Creating pull requests
- Accessing GitHub data through the MCP protocol

The GitHub MCP server will be automatically available when you use Claude Code commands that require GitHub integration.