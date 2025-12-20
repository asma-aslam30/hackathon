#!/usr/bin/env node
// github-mcp-server.js - A simple GitHub MCP server implementation

import { McpServer } from '@modelcontextprotocol/sdk/server/mcp.js';
import { createServer } from 'http';
import { Octokit } from '@octokit/rest';

// Create a new MCP server
const server = new McpServer({
  serverName: 'github-mcp-server',
  serverVersion: '1.0.0',
  capabilities: {
    tools: {
      listChanged: true,
    },
  },
});

// Add a GitHub client using the token from environment
const octokit = new Octokit({
  auth: process.env.GITHUB_TOKEN,
});

// Define a tool for listing repositories
server.tool('list-repositories', 'List repositories for the authenticated user', {
  type: 'object',
  properties: {
    username: {
      type: 'string',
      description: 'GitHub username to list repositories for (optional, defaults to authenticated user)',
    },
  },
  required: [],
}, async (input) => {
  try {
    const username = input?.username || null;

    if (username) {
      const response = await octokit.rest.repos.listForUser({
        username,
      });
      return {
        repositories: response.data.map(repo => ({
          name: repo.name,
          description: repo.description,
          url: repo.html_url,
          private: repo.private,
        })),
      };
    } else {
      const response = await octokit.rest.repos.listForAuthenticatedUser();
      return {
        repositories: response.data.map(repo => ({
          name: repo.name,
          description: repo.description,
          url: repo.html_url,
          private: repo.private,
        })),
      };
    }
  } catch (error) {
    return {
      error: error.message,
    };
  }
});

// Define a tool for creating an issue
server.tool('create-issue', 'Create a new issue in a repository', {
  type: 'object',
  properties: {
    owner: { type: 'string', description: 'Repository owner' },
    repo: { type: 'string', description: 'Repository name' },
    title: { type: 'string', description: 'Issue title' },
    body: { type: 'string', description: 'Issue body' },
  },
  required: ['owner', 'repo', 'title'],
}, async (input) => {
  try {
    const { owner, repo, title, body } = input;
    const response = await octokit.rest.issues.create({
      owner,
      repo,
      title,
      body,
    });
    return {
      success: true,
      issue: {
        number: response.data.number,
        title: response.data.title,
        url: response.data.html_url,
      },
    };
  } catch (error) {
    return {
      error: error.message,
    };
  }
});

// Define a tool for creating a pull request
server.tool('create-pull-request', 'Create a new pull request in a repository', {
  type: 'object',
  properties: {
    owner: { type: 'string', description: 'Repository owner' },
    repo: { type: 'string', description: 'Repository name' },
    title: { type: 'string', description: 'Pull request title' },
    head: { type: 'string', description: 'Head branch name' },
    base: { type: 'string', description: 'Base branch name' },
    body: { type: 'string', description: 'Pull request description' },
  },
  required: ['owner', 'repo', 'title', 'head', 'base'],
}, async (input) => {
  try {
    const { owner, repo, title, head, base, body } = input;
    const response = await octokit.rest.pulls.create({
      owner,
      repo,
      title,
      head,
      base,
      body,
    });
    return {
      success: true,
      pull_request: {
        number: response.data.number,
        title: response.data.title,
        url: response.data.html_url,
      },
    };
  } catch (error) {
    return {
      error: error.message,
    };
  }
});

// For MCP servers, we need to use the server's built-in transport handling
// The SDK will handle the transport based on environment (stdio, HTTP, etc.)
server.serve();