#!/usr/bin/env python3
"""
Test script to verify GitHub MCP server connectivity
"""

import os
import subprocess
import sys
import time

def test_github_mcp():
    """
    Test if the GitHub MCP server can be accessed
    """
    print("Testing GitHub MCP server connectivity...")

    # Check if GITHUB_TOKEN is set
    github_token = os.getenv('GITHUB_TOKEN')
    if not github_token or github_token == "your_github_personal_access_token_here":
        print("⚠️  WARNING: GITHUB_TOKEN is not set in environment variables.")
        print("   Please set your GitHub Personal Access Token in the .env file")
        print("   or export it as an environment variable.")
        return False

    try:
        # Check if the github-mcp-server.js file exists
        if not os.path.exists('github-mcp-server.js'):
            print("❌ GitHub MCP server file (github-mcp-server.js) not found.")
            return False

        # Try to run the GitHub MCP server to check if it starts properly
        result = subprocess.run([
            'node', 'github-mcp-server.js', '--help'
        ], capture_output=True, text=True, timeout=10)

        if result.returncode == 0 or 'github-mcp-server' in result.stdout.lower():
            print("✅ GitHub MCP server file is accessible!")
            return True
        else:
            # If --help doesn't work, try to start the server briefly
            print("Attempting to start GitHub MCP server...")
            process = subprocess.Popen([
                'node', 'github-mcp-server.js', '3001'
            ], env={**os.environ, 'GITHUB_TOKEN': github_token})

            # Wait a bit to see if it starts successfully
            time.sleep(3)

            # Check if the process is still running (indicating successful start)
            if process.poll() is None:
                print("✅ GitHub MCP server started successfully!")
                process.terminate()  # Stop the server
                return True
            else:
                print(f"❌ GitHub MCP server failed to start: {result.stderr}")
                return False

    except subprocess.TimeoutExpired:
        print("❌ GitHub MCP server test timed out")
        return False
    except FileNotFoundError:
        print("❌ 'node' command not found. Please ensure Node.js is installed.")
        return False
    except Exception as e:
        print(f"❌ Error testing GitHub MCP server: {str(e)}")
        return False

if __name__ == "__main__":
    success = test_github_mcp()
    sys.exit(0 if success else 1)