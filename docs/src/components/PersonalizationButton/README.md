# Chapter Personalization Feature

This feature allows logged-in users to personalize content in chapters based on their background. The system detects logged-in users, provides a 'Personalize' button at chapter start, fetches user profile from the backend, and adjusts content dynamically based on user's technical background (programming languages, tools, experience level, domain expertise).

## Components

### PersonalizationButton
- A React component that appears as a button in Docusaurus pages
- When clicked, fetches user profile and personalized content from backend
- Updates the chapter content dynamically using DOM manipulation

## How It Works

1. The button detects if a user is logged in by checking for an authentication token
2. On click, it fetches the user's profile from the backend API
3. It then requests personalized content based on the user's background
4. The chapter content is updated dynamically in the browser

## Usage in Docusaurus

To add the personalization button to a Docusaurus page, use the component:

```jsx
import PersonalizationButton from '@site/src/components/PersonalizationButton/PersonalizationButton';

<PersonalizationButton chapterId="unique-chapter-id" />
```

## Backend Integration

The frontend communicates with the backend via the following API endpoints:
- `GET /api/v1/profile` - Fetches user profile
- `GET /api/v1/personalization/chapter/{chapter_id}` - Gets personalized content
- Other endpoints for preferences, feedback, etc.

## Configuration

The API base URL can be configured using the `REACT_APP_API_BASE_URL` environment variable.