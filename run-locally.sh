#!/bin/bash

# Script to run the Docusaurus site locally with proper setup

echo "🚀 Setting up Docusaurus site for local development..."

# Change to docs directory
cd docs

# Install dependencies if not already installed
if [ ! -d "node_modules" ]; then
    echo "📦 Installing dependencies..."
    npm install
else
    echo "✅ Dependencies already installed"
fi

# Create placeholder images if they don't exist
if [ ! -d "static/img" ]; then
    echo "🖼️ Creating static/img directory..."
    mkdir -p static/img
fi

# Create placeholder SVG images if they don't exist
if [ ! -f "static/img/hero-illustration.svg" ]; then
    echo '<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="#e2e8f0"/><text x="200" y="150" font-family="Arial" font-size="16" text-anchor="middle" fill="#4a5568">Homepage Image</text></svg>' > static/img/hero-illustration.svg
    echo "🖼️ Created hero-illustration.svg"
fi

if [ ! -f "static/img/hero-slide-1.svg" ]; then
    echo '<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="#feb2b2"/><text x="200" y="150" font-family="Arial" font-size="16" text-anchor="middle" fill="#4a5568">Slide 1</text></svg>' > static/img/hero-slide-1.svg
    echo "🖼️ Created hero-slide-1.svg"
fi

if [ ! -f "static/img/hero-slide-2.svg" ]; then
    echo '<svg xmlns="http://www.w3.org/2000/svg" width="400" height="300" viewBox="0 0 400 300"><rect width="400" height="300" fill="#fed7d7"/><text x="200" y="150" font-family="Arial" font-size="16" text-anchor="middle" fill="#4a5568">Slide 2</text></svg>' > static/img/hero-slide-2.svg
    echo "🖼️ Created hero-slide-2.svg"
fi

if [ ! -f "static/img/icon-guide.svg" ]; then
    echo '<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"><circle cx="50" cy="50" r="40" fill="#90cdf4"/></svg>' > static/img/icon-guide.svg
    echo "🖼️ Created icon-guide.svg"
fi

if [ ! -f "static/img/icon-example.svg" ]; then
    echo '<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"><circle cx="50" cy="50" r="40" fill="#68d391"/></svg>' > static/img/icon-example.svg
    echo "🖼️ Created icon-example.svg"
fi

if [ ! -f "static/img/icon-ai.svg" ]; then
    echo '<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"><circle cx="50" cy="50" r="40" fill="#fbbf24"/></svg>' > static/img/icon-ai.svg
    echo "🖼️ Created icon-ai.svg"
fi

# Start the development server
echo "🌐 Starting Docusaurus development server..."
echo "The site will be available at http://localhost:3000"
echo ""
echo "💡 Press Ctrl+C to stop the server"
echo ""

npm start