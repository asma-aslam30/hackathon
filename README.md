# Physical AI & Humanoid Robotics Platform

This project provides a comprehensive guide to Physical AI & Humanoid Robotics with an integrated chatbot for interactive learning.

## 🚀 Features

- **Comprehensive Documentation**: Detailed guides on Physical AI & Robotics
- **AI-Powered Chatbot**: Interactive assistant to answer questions about the content
- **RAG System**: Retrieval-Augmented Generation for accurate responses
- **Modern UI**: Built with Docusaurus for an excellent user experience
- **Interactive Components**: Cybernetic and futuristic design elements

## 🏗️ Architecture

The system consists of:

- **Frontend**: Docusaurus-based documentation site with integrated chatbot
- **Backend**: FastAPI service with AI query capabilities
- **Vector Database**: Qdrant for storing and retrieving embeddings
- **AI Provider**: OpenAI or Google Gemini for natural language processing

## 🛠️ Prerequisites

- Docker and Docker Compose
- Node.js (v18 or higher)
- npm or yarn
- API keys for AI provider (OpenAI or Google Gemini)
- Qdrant Cloud account or local Qdrant instance

## 📦 Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup

```bash
cd backend
cp .env.example .env
# Edit .env with your API keys
```

### 3. Frontend Setup

```bash
cd docs
npm install
```

## 🚀 Deployment

### Option 1: Local Development

**Start Backend:**
```bash
cd backend
docker-compose up --build
```

**Start Frontend:**
```bash
cd docs
npm run start
```

The frontend will be available at `http://localhost:3000`
The backend API will be available at `http://localhost:8000`

### Option 2: Production Deployment

**Backend:**
```bash
cd backend
./deploy.sh
```

**Frontend:**
```bash
cd docs
npm run build
npm run serve  # For local serving, or deploy build/ folder to your hosting
```

## 🔧 Configuration

### Backend Environment Variables (.env)

```bash
# AI Provider (gemini or openai)
AI_PROVIDER=gemini

# Gemini Configuration
GEMINI_API_KEY=your_api_key_here
GEMINI_MODEL=gemini-2.5-flash

# OpenAI Configuration (alternative to Gemini)
OPENAI_API_KEY=your_api_key_here
OPENAI_MODEL=gpt-4-turbo-preview

# Qdrant Configuration
QDRANT_API_KEY=your_api_key_here
QDRANT_HOST=your_cluster_url
QDRANT_COLLECTION_NAME=rag_embeddings

# Additional Settings
COHERE_API_KEY=your_cohere_api_key_here
TARGET_URL=https://your-frontend-domain.com
```

### Frontend Environment Variables

The frontend automatically connects to the backend:
- In development: connects to `http://localhost:8000`
- In production: connects to the same origin

To override, set `REACT_APP_API_BASE_URL` during build:
```bash
REACT_APP_API_BASE_URL=https://your-backend.com npm run build
```

## 🤖 Chatbot Integration

The chatbot is integrated into the frontend and connects to the backend API at `/api/v1/query`. It provides:

- Natural language queries about the documentation
- Source citations for provided answers
- Confidence scores for responses
- Real-time interaction with the RAG system

### UI Enhancements
- Modern gradient animations and interactive elements
- 3D card effects with hover animations
- Progressive disclosure elements
- Animated progress indicators
- Interactive code blocks with copy functionality
- Responsive design optimized for all screen sizes

### Interactive Components
- InteractiveCard components with multiple variants (glass, 3D, gradient, magnetic, neon)
- BentoGrid layout system
- Animated backgrounds and visual effects
- Scroll-triggered animations
- Theme-aware components

### Cybernetic Design Elements
- Holographic UI effects
- Cyberpunk-inspired visuals
- Robotic simulation visualizations
- Terminal-style code blocks
- Neural network visualizations
- Data flow animations

## Technical Implementation

### CSS Modules
- Custom CSS with modern animations and gradients
- Modular component styles with CSS modules
- Responsive design patterns
- Dark/light theme support

### JavaScript Functionality
- Interactive UI elements
- Scroll-triggered animations
- Progress indicators
- Theme enhancements
- Accessibility features

### Component Architecture
- Reusable React components
- TypeScript type safety
- Modular component structure
- Integration with Docusaurus ecosystem

## File Structure

```
.
├── backend/                 # FastAPI backend service
│   ├── src/
│   │   ├── api/            # API routes and models
│   │   ├── agents/         # AI agent implementations
│   │   ├── services/       # Core services
│   │   └── config/         # Configuration
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── requirements.txt
│   └── deploy.sh           # Deployment script
├── docs/                   # Docusaurus frontend
│   ├── src/
│   │   ├── components/     # React components
│   │   │   ├── Carousel/
│   │   │   ├── HomepageFeatures/
│   │   │   ├── InteractiveCard/  # Interactive card component
│   │   │   ├── BentoGrid/        # Grid layout component
│   │   │   ├── AnimatedBackground/
│   │   │   ├── ImageShowcase/
│   │   │   ├── StatsCounter/
│   │   │   └── Chatbot/          # AI chatbot component
│   │   ├── css/
│   │   │   ├── custom.css       # Main custom styles
│   │   │   └── cyber-ui-enhancements.css # Cybernetic UI enhancements
│   │   ├── pages/               # Homepage and other pages
│   │   └── theme/               # Custom theme components
│   ├── static/js/               # Client-side JavaScript
│   │   └── interactive-ui.js    # Interactive UI functionality
│   └── docusaurus.config.ts     # Docusaurus configuration
├── DEPLOYMENT.md               # Detailed deployment guide
├── test-deployment.sh          # Deployment testing script
└── README.md                   # This file
```

## 🧪 Testing

To test the complete deployment:

```bash
./test-deployment.sh
```

## 📚 Available Pages

- **Homepage**: `/` - Main landing page
- **Documentation**: `/docs` - Complete Physical AI & Robotics guide
- **Chatbot**: `/chatbot` - AI-powered assistant
- **API Docs**: `/docs` (backend) - Interactive API documentation

## 🔒 Security

- API keys are stored in environment variables
- CORS configured for secure cross-origin requests
- Input validation and sanitization
- Rate limiting (can be configured)

## 📈 Scaling

- Backend uses multiple workers for concurrent requests
- Qdrant can be scaled independently
- Frontend is static and CDN-friendly
- Docker Compose supports multiple instances

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 🧩 GitHub MCP Integration

This project now includes a GitHub MCP (Model Context Protocol) server that enables enhanced interaction with GitHub repositories. The GitHub MCP server provides capabilities for:

- Repository operations (clone, pull, push, branch management)
- Pull request management (create, review, comment, merge)
- Issue tracking (create, list, comment, close/reopen)
- GitHub Actions integration (view, trigger, monitor workflows)
- Code search and file operations
- Release management

### Setup GitHub MCP

1. Create a GitHub Personal Access Token with appropriate permissions
2. Add your token to the `.env` file:
   ```bash
   GITHUB_TOKEN=your_github_personal_access_token_here
   ```
3. The MCP configuration is already set up in `mcp-config.json`
4. Test the integration with:
   ```bash
   python3 test_github_mcp.py
   ```

For complete setup instructions, see `GITHUB_MCP_README.md`.

## 📄 License

This project is licensed under the MIT License.

## 🆘 Support

For support, please open an issue in the GitHub repository.

---

Made with ❤️ using Docusaurus and FastAPI