# Data Model: User Authentication & Background Collection

## Entities

### User Account
- **id**: UUID (Primary Key)
- **email**: String (Unique, Required, Validated)
- **name**: String (Optional)
- **avatar_url**: String (Optional)
- **created_at**: DateTime (Auto-generated)
- **updated_at**: DateTime (Auto-generated)
- **last_login_at**: DateTime (Optional, Updated on login)
- **is_active**: Boolean (Default: true)
- **email_verified**: Boolean (Default: false)

### Background Information
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User Account, Unique per user)
- **software_tools**: JSONB (Array of strings representing software tools)
- **hardware_setup**: JSONB (Object describing hardware configuration)
- **programming_languages**: JSONB (Array of strings representing programming languages)
- **technical_preferences**: JSONB (Object with technical preferences)
- **experience_level**: String (Enum: beginner, intermediate, advanced, expert)
- **primary_domain**: String (Optional, e.g., web, mobile, data, ai)
- **created_at**: DateTime (Auto-generated)
- **updated_at**: DateTime (Auto-generated)

### Authentication Session
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User Account)
- **session_token**: String (Unique, Indexed)
- **expires_at**: DateTime (Required)
- **created_at**: DateTime (Auto-generated)
- **last_accessed_at**: DateTime (Auto-generated)
- **ip_address**: String (Optional, for security)
- **user_agent**: String (Optional, for security)

### Personalization Profile
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User Account, Unique per user)
- **recommended_content**: JSONB (Array of content recommendations)
- **preferred_categories**: JSONB (Array of category preferences)
- **last_personalization_update**: DateTime (Auto-generated)
- **created_at**: DateTime (Auto-generated)
- **updated_at**: DateTime (Auto-generated)

## Relationships
- User Account (1) ←→ (0 or 1) Background Information (via user_id)
- User Account (1) ←→ (0 to many) Authentication Session (via user_id)
- User Account (1) ←→ (0 or 1) Personalization Profile (via user_id)

## Validation Rules
- User email must be a valid email format
- Background information fields must be properly formatted JSON
- Session tokens must be unique and expire within 30 days
- All timestamps are stored in UTC

## Indexes
- User Account: email (unique), created_at
- Background Information: user_id (unique), updated_at
- Authentication Session: session_token (unique), expires_at
- Personalization Profile: user_id (unique)