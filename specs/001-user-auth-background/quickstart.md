# Quickstart Guide: User Authentication & Background Collection

## Prerequisites
- Python 3.11+
- Node.js 18+
- Neon Postgres database instance
- Better-Auth compatible environment

## Setup

### 1. Environment Configuration
```bash
# Copy environment template
cp .env.example .env

# Update with your values
NEON_DATABASE_URL="postgresql://username:password@ep-..."
BETTER_AUTH_SECRET="your-secret-key-here"
NEXT_PUBLIC_SITE_URL="http://localhost:3000"
```

### 2. Backend Setup
```bash
cd backend
pip install -r requirements.txt

# Run database migrations
python -m src.main migrate

# Start backend server
python -m src.main dev
```

### 3. Frontend Setup
```bash
cd frontend
npm install

# Start development server
npm run dev
```

## API Endpoints

### Authentication
- `POST /api/auth/register` - Create new user account
- `POST /api/auth/login` - Authenticate user
- `POST /api/auth/logout` - End user session
- `POST /api/auth/forgot-password` - Password reset request
- `POST /api/auth/reset-password` - Set new password

### Background Information
- `GET /api/background/me` - Get user's background info
- `POST /api/background/me` - Create/update background info
- `DELETE /api/background/me` - Remove background info

### Profile Management
- `GET /api/profile/me` - Get user profile
- `PUT /api/profile/me` - Update profile
- `DELETE /api/profile/me` - Deactivate account

## Integration with Other Agents

### For Personalization Agent
```javascript
// Example: Fetch user background for personalization
const getUserBackground = async (userId) => {
  const response = await fetch(`/api/background/${userId}`, {
    headers: { 'Authorization': `Bearer ${agentToken}` }
  });
  return response.json();
};
```

### MCP Integration
The API endpoints follow MCP standards:
- All endpoints return structured JSON responses
- Error responses include appropriate HTTP status codes
- Authentication uses standard Bearer token approach

## Development

### Running Tests
```bash
# Backend tests
cd backend
python -m pytest tests/

# Frontend tests
cd frontend
npm run test
```

### Database Migrations
```bash
# Create new migration
python -m src.main create-migration "add_user_preferences"

# Apply migrations
python -m src.main migrate
```

## Configuration

### Settings
- `BETTER_AUTH_SECRET`: Secret key for authentication tokens
- `NEON_DATABASE_URL`: Connection string for Neon Postgres
- `SESSION_EXPIRY_DAYS`: Session duration in days (default: 7)
- `MAX_BACKGROUND_SIZE`: Maximum size for background data (default: 10KB)

## Security Considerations
- All authentication endpoints require HTTPS in production
- Passwords are automatically hashed by Better-Auth
- Session tokens have configurable expiry times
- Rate limiting applied to authentication endpoints
- Input validation on all user-provided data