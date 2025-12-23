# Data Model: Chapter Personalization

## Overview
This document defines the data models for the Chapter Personalization feature, including entities, relationships, and validation rules derived from the functional requirements.

## Entity: UserBackgroundProfile
**Source**: Integrated from UserAuthAgent system
**Description**: Contains user's technical background information that drives personalization

### Fields
- `user_id` (UUID, Primary Key): Reference to the user account
- `programming_languages` (Array<String>): List of programming languages the user is familiar with
- `experience_level` (String): Enum (beginner, intermediate, advanced, expert)
- `domain_expertise` (Array<String>): List of domains/fields the user has experience in
- `tools_familiarity` (Array<String>): List of tools and frameworks the user is familiar with
- `os_familiarity` (Array<String>): List of operating systems the user is comfortable with
- `created_at` (DateTime): Timestamp when profile was created
- `updated_at` (DateTime): Timestamp when profile was last updated

### Validation Rules
- `experience_level` must be one of the defined enum values
- `programming_languages` must contain valid language identifiers
- `domain_expertise` must contain valid domain identifiers

## Entity: PersonalizationPreferences
**Description**: Stores user-specific settings for personalization intensity and types of adaptations

### Fields
- `id` (UUID, Primary Key): Unique identifier for the preference record
- `user_id` (UUID): Reference to the user account
- `intensity` (String): Enum (low, medium, high) - how much personalization to apply
- `customization_types` (JSON): Object containing boolean flags for different customization types
  - `examples`: Whether to customize examples based on user background
  - `complexity`: Whether to adjust content complexity
  - `domain_focus`: Whether to focus content on user's domain expertise
- `created_at` (DateTime): Timestamp when preferences were created
- `updated_at` (DateTime): Timestamp when preferences were last updated

### Validation Rules
- `intensity` must be one of the defined enum values
- `user_id` must reference an existing user
- `customization_types` must be a valid JSON object with expected structure

## Entity: PersonalizationRule
**Description**: Contains the mapping logic between user background attributes and content adaptation strategies

### Fields
- `id` (UUID, Primary Key): Unique identifier for the rule
- `name` (String): Descriptive name for the rule
- `condition` (JSON): Conditions that determine when to apply the rule
- `action` (JSON): What transformation to apply when conditions are met
- `priority` (Integer): Order in which rules should be applied
- `enabled` (Boolean): Whether the rule is currently active
- `created_at` (DateTime): Timestamp when rule was created
- `updated_at` (DateTime): Timestamp when rule was last updated

### Validation Rules
- `priority` must be a positive integer
- `condition` and `action` must be valid JSON objects
- `enabled` must be a boolean value

## Entity: ContentAdaptation
**Description**: Represents the actual modifications made to original content

### Fields
- `id` (UUID, Primary Key): Unique identifier for the adaptation
- `chapter_id` (String): Reference to the chapter being adapted
- `user_id` (UUID): Reference to the user for whom content is adapted
- `original_content_id` (String): Reference to the original content
- `adapted_content` (Text): The personalized version of the content
- `personalization_metadata` (JSON): Information about what aspects of user profile were used
- `created_at` (DateTime): Timestamp when adaptation was created
- `cache_expires_at` (DateTime): When the adaptation should be refreshed

### Validation Rules
- `user_id` must reference an existing user
- `chapter_id` must reference an existing chapter
- `original_content_id` must reference an existing content item

## Entity: PersonalizationFeedback
**Description**: Stores user feedback on the effectiveness of personalization

### Fields
- `id` (UUID, Primary Key): Unique identifier for the feedback
- `user_id` (UUID): Reference to the user providing feedback
- `chapter_id` (String): Reference to the chapter being evaluated
- `feedback_type` (String): Enum (positive, negative, neutral)
- `feedback_text` (Text): Optional detailed feedback from user
- `created_at` (DateTime): Timestamp when feedback was submitted

### Validation Rules
- `feedback_type` must be one of the defined enum values
- `user_id` must reference an existing user
- `chapter_id` must reference an existing chapter

## Relationships
- `UserBackgroundProfile` 1-to-1 with `PersonalizationPreferences` (user has one set of preferences)
- `UserBackgroundProfile` 1-to-many with `ContentAdaptation` (user can have many adaptations)
- `UserBackgroundProfile` 1-to-many with `PersonalizationFeedback` (user can provide many feedbacks)
- `PersonalizationRule` 1-to-many with `ContentAdaptation` (rule can be used in many adaptations)

## State Transitions
- `PersonalizationPreferences`: Created when user first interacts with personalization, updated when user changes settings
- `ContentAdaptation`: Created when content is personalized for a user, expires based on cache policy
- `PersonalizationFeedback`: Created when user submits feedback, immutable after creation