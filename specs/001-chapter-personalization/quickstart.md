# Quickstart: Chapter Personalization

## Overview
This guide provides a quick introduction to implementing and using the Chapter Personalization feature. The feature allows logged-in users to personalize content in chapters based on their background.

## Prerequisites
- User authentication system (UserAuthAgent) must be operational
- Chapter content must be available in the system
- Background profile information must be collected from users

## Getting Started

### 1. Detect Logged-in User
The system automatically detects when a user is logged in by checking for valid authentication tokens in API requests.

### 2. Add 'Personalize' Button
A 'Personalize' button is added at the beginning of each chapter that is visible to logged-in users. This button allows users to toggle personalization on/off.

### 3. Fetch User Profile
When personalization is requested, the system fetches the user's profile from the UserAuthAgent system, including:
- Programming languages familiarity
- Experience level
- Domain expertise
- Tools familiarity

### 4. Adjust Content Dynamically
Based on the user's profile, the system dynamically adjusts the content by:
- Customizing examples to match the user's programming languages
- Adjusting complexity based on experience level
- Focusing on relevant domains of expertise
- Modifying content presentation style

## API Usage Examples

### Get Personalized Chapter Content
```bash
curl -X GET \
  http://localhost:8000/api/v1/personalization/chapter/{chapter_id} \
  -H "Authorization: Bearer {user_token}"
```

### Update Personalization Preferences
```bash
curl -X PUT \
  http://localhost:8000/api/v1/personalization/preferences \
  -H "Authorization: Bearer {user_token}" \
  -H "Content-Type: application/json" \
  -d '{
    "intensity": "medium",
    "customization_types": {
      "examples": true,
      "complexity": true,
      "domain_focus": true
    }
  }'
```

## Frontend Integration

### Personalization Button Component
The frontend includes a `PersonalizationButton` component that:
- Detects user authentication status
- Fetches user profile information
- Communicates with the personalization API
- Updates the chapter content dynamically

### Content Rendering
The `ChapterContent` component renders personalized content by:
- Requesting personalized content from the API
- Handling fallback to original content if personalization fails
- Caching personalized content for performance

## Testing the Feature

### Unit Tests
- Test personalization rule application
- Test content transformation functions
- Test user profile integration

### Integration Tests
- Test end-to-end personalization flow
- Test API endpoints with various user profiles
- Test performance with concurrent users

## Configuration

### Environment Variables
- `PERSONALIZATION_CACHE_TTL`: Time-to-live for cached personalized content (default: 300 seconds)
- `PERSONALIZATION_MAX_CONCURRENT_USERS`: Maximum concurrent personalization requests (default: 1000)
- `PERSONALIZATION_DEBUG`: Enable debug logging (default: false)

### Feature Flags
- `personalization_enabled`: Enable/disable the entire personalization feature
- `personalization_preferences_enabled`: Enable/disable user preference controls
- `personalization_analytics_enabled`: Enable/disable feedback collection