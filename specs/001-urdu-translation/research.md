# Research: Urdu Translation Feature

**Feature**: 001-urdu-translation | **Date**: 2025-12-20 | **Status**: Complete

## Overview

This document captures research findings and resolved decisions for implementing the Urdu translation feature for logged-in users.

---

## 1. Translation API Provider Selection

### Decision: OpenAI GPT-4 API (Primary) with Fallback

### Rationale
- OpenAI is already integrated in the backend (`openai==1.3.5` in requirements.txt)
- GPT-4 provides high-quality translation for Urdu with proper Nastaliq script support
- Existing `config/settings.py` already loads `OPENAI_API_KEY` from environment
- Claude API available as fallback via existing Google Generative AI integration pattern

### Alternatives Considered
| Alternative | Pros | Cons | Rejected Because |
|-------------|------|------|------------------|
| Google Translate API | Fast, reliable, cost-effective | Lower quality for Urdu literary content | Technical terminology translation quality insufficient |
| Microsoft Azure Translator | Good Urdu support | Additional SDK integration required | More complexity, OpenAI already available |
| DeepL | High quality translations | No Urdu support | Does not support Urdu language |
| Custom model | Full control | Expensive, training required | Out of scope, unnecessary complexity |

### Implementation Details
```python
# Prompt template for Urdu translation
URDU_TRANSLATION_PROMPT = """
Translate the following content to Urdu while:
1. Preserving HTML structure and formatting
2. Maintaining technical terminology accuracy
3. Using proper Nastaliq script
4. Adapting idioms to Urdu equivalents
5. Keeping code blocks and inline code unchanged

Content to translate:
{content}
"""
```

---

## 2. RTL (Right-to-Left) Text Rendering

### Decision: CSS-based RTL with Noto Nastaliq Urdu Font

### Rationale
- CSS `direction: rtl` is the standard approach for RTL languages
- Noto Nastaliq Urdu is a free, high-quality Google Font designed specifically for Urdu
- Browser support is excellent (all modern browsers)
- Minimal JavaScript needed - CSS handles most layout concerns

### Implementation Details
```css
/* RTL container for Urdu content */
.urdu-content {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif;
  line-height: 2.2;  /* Nastaliq requires extra line height */
  font-size: 1.2em;  /* Slightly larger for readability */
}

/* Preserve LTR for code blocks within RTL content */
.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
  unicode-bidi: isolate;
}
```

### Font Loading Strategy
- Use Google Fonts CDN for Noto Nastaliq Urdu
- Fallback to system Arabic/Urdu fonts
- Font-display: swap for performance

---

## 3. Caching Strategy

### Decision: PostgreSQL-based Cache with 24-hour TTL

### Rationale
- Already using PostgreSQL (Neon) for all persistent data
- Simpler than adding Redis (no additional infrastructure)
- Translation content is stable (doesn't change frequently)
- 24-hour TTL balances cache freshness with API cost savings
- Existing SQLAlchemy patterns make implementation straightforward

### Alternatives Considered
| Alternative | Pros | Cons | Rejected Because |
|-------------|------|------|------------------|
| Redis | Fast, in-memory | Additional service to manage | Over-engineering for current scale |
| In-memory (Python dict) | Simple, fast | Lost on restart, no sharing | Not persistent across deployments |
| File-based cache | Simple persistence | Slow, disk I/O | Performance concerns |
| No caching | Simplest | High API costs, slow UX | Would violate SC-001 (3s target) |

### Cache Key Strategy
```python
# Cache key format: hash of content + source language + target language
cache_key = hashlib.sha256(f"{source_lang}:{target_lang}:{content}".encode()).hexdigest()
```

---

## 4. Authentication Integration

### Decision: Leverage Existing JWT BetterAuth System

### Rationale
- Authentication system already implemented (`backend/src/auth/better_auth.py`)
- Frontend `authService.js` provides token management
- Translation endpoint will use existing `get_current_user` dependency
- No new authentication logic required

### Implementation Pattern
```python
# Backend endpoint pattern
@router.post("/translate-to-urdu")
async def translate_to_urdu(
    request: TranslationRequest,
    current_user: User = Depends(get_current_user),  # Existing auth
    db: AsyncSession = Depends(get_db)
):
    # Only authenticated users can translate
    ...
```

```javascript
// Frontend pattern - check auth before showing button
const { isAuthenticated } = useAuth();

return isAuthenticated && (
  <UrduTranslationButton onClick={handleTranslate} />
);
```

---

## 5. Component Architecture

### Decision: Follow Existing PersonalizationButton Pattern

### Rationale
- `PersonalizationButton` already implements similar functionality:
  - Auth-gated visibility
  - Toggle between original and transformed content
  - Loading states and error handling
- Consistent UX across features
- Proven pattern in the codebase

### Component Structure
```
UrduTranslationButton/
├── index.js           # Main component with auth check
├── styles.module.css  # RTL-aware styling
└── hooks/
    └── useTranslation.js  # Translation state management
```

### State Management
```javascript
const [translationState, setTranslationState] = useState({
  isTranslated: false,
  isLoading: false,
  error: null,
  originalContent: null,
  translatedContent: null
});
```

---

## 6. Content Processing

### Decision: Server-side Translation with HTML Preservation

### Rationale
- Server handles AI API calls (keeps API keys secure)
- HTML structure preserved through careful prompt engineering
- Client receives ready-to-render Urdu HTML
- Code blocks excluded from translation

### Processing Flow
1. Client sends chapter content (HTML string)
2. Server extracts translatable text segments
3. Server sends text to OpenAI for translation
4. Server reconstructs HTML with Urdu text
5. Server caches result and returns to client
6. Client renders with RTL direction

### Content Segments to Handle
| Segment Type | Translation Behavior |
|--------------|---------------------|
| Paragraph text | Full translation |
| Headings | Full translation |
| List items | Full translation |
| Code blocks (`<pre>`, `<code>`) | Preserve original (no translation) |
| Inline code | Preserve original |
| Links | Translate link text, preserve href |
| Images | Translate alt text only |
| Tables | Translate cell content |

---

## 7. Error Handling

### Decision: Graceful Degradation with User Feedback

### Rationale
- Users should never be blocked from reading content
- Clear error messages improve UX
- Original content always accessible
- Aligns with FR-006 from spec

### Error Scenarios and Responses
| Error Type | User Message | Behavior |
|------------|--------------|----------|
| API rate limit | "Translation service busy. Please try again in a moment." | Retain original, show retry button |
| API timeout | "Translation taking longer than expected. Please try again." | Retain original, show retry |
| Invalid content | "Unable to translate this content type." | Retain original |
| Auth expired | "Please log in to use translation." | Redirect to login |
| Network error | "Connection error. Check your internet and try again." | Retain original |

---

## 8. Performance Optimization

### Decision: Progressive Loading with Chunked Translation

### Rationale
- Long chapters may exceed 3s target
- Progressive rendering improves perceived performance
- Chunking prevents API timeouts

### Implementation Strategy
1. Split content into ~1000 word chunks
2. Translate chunks in parallel (max 3 concurrent)
3. Render each chunk as it completes
4. Show progress indicator for remaining chunks

### Performance Targets
| Metric | Target | Measurement |
|--------|--------|-------------|
| Initial response | <500ms | Time to loading indicator |
| Full translation (< 5000 words) | <3s | SC-001 requirement |
| Large chapter (> 5000 words) | <8s | Progressive loading helps |

---

## 9. Accessibility Requirements

### Decision: WCAG 2.1 AA Compliance

### Rationale
- FR-010 mandates accessibility compliance
- Urdu-speaking users may include visually impaired users
- Proper ARIA labels ensure screen reader compatibility

### Accessibility Features
| Feature | Implementation |
|---------|----------------|
| Button label | `aria-label="Translate to Urdu"` with Urdu equivalent |
| Loading state | `aria-busy="true"`, `aria-live="polite"` announcement |
| Language switch | `lang="ur"` attribute on translated content |
| Keyboard access | Tab-focusable, Enter/Space to activate |
| Screen reader | Announce "Content translated to Urdu" on completion |

---

## 10. Testing Strategy

### Decision: Multi-layer Testing Approach

### Unit Tests (Backend)
- Translation service mocks OpenAI API
- Cache hit/miss scenarios
- HTML preservation logic
- Error handling paths

### Integration Tests (Backend)
- Full translation endpoint with test content
- Auth requirement enforcement
- Cache behavior verification

### Component Tests (Frontend)
- Button visibility based on auth state
- Loading state rendering
- RTL content display
- Error message rendering

### E2E Tests
- Full user flow: login → view chapter → translate → revert
- Performance testing with various content sizes

---

## Summary of Resolved Decisions

| Decision | Choice | Confidence |
|----------|--------|------------|
| Translation API | OpenAI GPT-4 | High |
| RTL rendering | CSS direction + Noto Nastaliq Urdu | High |
| Caching | PostgreSQL with 24h TTL | High |
| Authentication | Existing BetterAuth JWT | High |
| Component pattern | Follow PersonalizationButton | High |
| Content processing | Server-side with HTML preservation | High |
| Error handling | Graceful degradation | High |
| Performance | Progressive chunked loading | Medium |
| Accessibility | WCAG 2.1 AA | High |
| Testing | Unit + Integration + E2E | High |

All NEEDS CLARIFICATION items have been resolved. Ready for Phase 1: Design & Contracts.
