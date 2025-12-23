# Quickstart: Urdu Translation Feature

**Feature**: 001-urdu-translation | **Date**: 2025-12-20

## Overview

This guide helps developers get started with implementing and testing the Urdu translation feature.

---

## Prerequisites

### Environment Setup

1. **Python 3.11+** with pip
2. **Node.js 18+** with npm
3. **PostgreSQL** database (Neon instance)
4. **OpenAI API key** for translation

### Required Environment Variables

Add to your `.env` file:

```bash
# Existing
DATABASE_URL=postgresql+asyncpg://user:pass@host/db
OPENAI_API_KEY=sk-your-key-here

# New for translation (optional - has defaults)
TRANSLATION_CACHE_TTL_HOURS=24
TRANSLATION_MAX_CONTENT_LENGTH=100000
TRANSLATION_RATE_LIMIT_PER_MINUTE=10
```

---

## Backend Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

New dependencies (add to `requirements.txt` if not present):
```
openai>=1.3.5
```

### 2. Run Database Migration

```bash
# Apply migration for translation_cache table
alembic upgrade head

# Or manually run SQL
psql $DATABASE_URL -f migrations/001_add_translation_cache.sql
```

### 3. Start Backend Server

```bash
cd backend
uvicorn src.main:app --reload --port 8000
```

### 4. Verify Translation Endpoint

```bash
# Get auth token first
TOKEN=$(curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass"}' \
  | jq -r '.session_token')

# Test translation
curl -X POST http://localhost:8000/api/v1/translation/translate-to-urdu \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"content":"<p>Hello, world!</p>","source_language":"en"}'
```

Expected response:
```json
{
  "success": true,
  "translated_content": "<p dir='rtl' lang='ur'>ہیلو، دنیا!</p>",
  "metadata": {
    "word_count": 2,
    "translation_time_ms": 1250,
    "source_language": "en",
    "target_language": "ur",
    "cached_at": null
  },
  "cache_hit": false
}
```

---

## Frontend Setup (Docusaurus)

### 1. Install Dependencies

```bash
cd docs
npm install
```

### 2. Add Google Fonts for Urdu

In `docs/docusaurus.config.ts`, add to `head` array:

```typescript
head: [
  // ... existing entries
  {
    tagName: 'link',
    attributes: {
      rel: 'preconnect',
      href: 'https://fonts.googleapis.com',
    },
  },
  {
    tagName: 'link',
    attributes: {
      rel: 'stylesheet',
      href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu&display=swap',
    },
  },
],
```

### 3. Start Docusaurus Dev Server

```bash
cd docs
npm start
```

### 4. Test Translation Button

1. Navigate to any chapter page
2. Log in with test credentials
3. Click "Translate to Urdu" button
4. Verify content displays in Urdu with RTL layout

---

## Component Usage

### UrduTranslationButton

Import and use in any React component:

```jsx
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

function ChapterPage({ content }) {
  const [displayContent, setDisplayContent] = useState(content);
  const [isTranslated, setIsTranslated] = useState(false);

  return (
    <div>
      <UrduTranslationButton
        content={content}
        onTranslate={(translatedContent) => {
          setDisplayContent(translatedContent);
          setIsTranslated(true);
        }}
        onRevert={() => {
          setDisplayContent(content);
          setIsTranslated(false);
        }}
        isTranslated={isTranslated}
      />

      <div
        className={isTranslated ? 'urdu-content' : ''}
        dangerouslySetInnerHTML={{ __html: displayContent }}
      />
    </div>
  );
}
```

### CSS Classes

Available classes for RTL styling:

```css
/* Apply to container when showing Urdu content */
.urdu-content {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', serif;
  line-height: 2.2;
}

/* Preserve LTR for code blocks */
.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
}
```

---

## API Reference

### POST /api/v1/translation/translate-to-urdu

Translate content to Urdu.

**Headers:**
- `Authorization: Bearer <token>` (required)
- `Content-Type: application/json`

**Request Body:**
```json
{
  "content": "<p>Your HTML content here</p>",
  "chapter_id": "optional-chapter-id",
  "source_language": "en",
  "preserve_formatting": true
}
```

**Response (200):**
```json
{
  "success": true,
  "translated_content": "<p dir='rtl' lang='ur'>آپ کا HTML مواد یہاں</p>",
  "metadata": {
    "word_count": 4,
    "translation_time_ms": 1500,
    "source_language": "en",
    "target_language": "ur",
    "cached_at": null
  },
  "cache_hit": false
}
```

**Error Responses:**
- `401` - Not authenticated
- `429` - Rate limited (check `retry_after`)
- `500` - Service error

---

## Testing

### Backend Unit Tests

```bash
cd backend
pytest tests/unit/test_translation_service.py -v
```

### Backend Integration Tests

```bash
cd backend
pytest tests/integration/test_translation_api.py -v
```

### Frontend Component Tests

```bash
cd docs
npm test -- --testPathPattern=UrduTranslationButton
```

### E2E Tests

```bash
cd docs
npm run test:e2e
```

---

## Common Issues

### Issue: Translation takes too long

**Cause:** Large content or cold API call
**Solution:**
- Content is chunked automatically for large documents
- First request is slower; subsequent requests use cache

### Issue: Urdu text not displaying correctly

**Cause:** Missing font
**Solution:** Ensure Noto Nastaliq Urdu font is loaded in document head

### Issue: Code blocks translated

**Cause:** HTML structure not preserved
**Solution:** Ensure `preserve_formatting: true` in request

### Issue: 429 Rate Limited

**Cause:** Too many requests per minute
**Solution:** Wait for `retry_after` seconds; cache should reduce frequency

---

## Architecture Overview

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Docusaurus    │────▶│   FastAPI       │────▶│   OpenAI API    │
│   Frontend      │     │   Backend       │     │   Translation   │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                                 ▼
                        ┌─────────────────┐
                        │   PostgreSQL    │
                        │   Cache         │
                        └─────────────────┘
```

**Flow:**
1. User clicks "Translate to Urdu"
2. Frontend sends content to `/api/v1/translation/translate-to-urdu`
3. Backend checks cache for existing translation
4. If cache miss, sends to OpenAI for translation
5. Stores result in cache with 24h TTL
6. Returns translated content with RTL metadata
7. Frontend renders with RTL direction

---

## Next Steps

After completing quickstart setup:

1. Run `/sp.tasks` to generate implementation tasks
2. Implement backend translation service
3. Implement frontend components
4. Write tests
5. Deploy and validate with linguistic review
