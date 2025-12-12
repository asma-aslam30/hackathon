---
description: Display help information for all available Specify system commands.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

If `$ARGUMENTS` is empty or "help", display the complete help information for all available commands. If `$ARGUMENTS` specifies a particular command (e.g., "specify", "analyze", "constitution"), provide detailed help for just that command.

## Available Commands

The Specify system provides these commands for AI-assisted development:

### `/sp.specify` - Feature Specification
**Purpose**: Create or update feature specifications from natural language descriptions
**Usage**: `/sp.specify [feature description]`
**What it does**:
- Analyzes your feature description to extract requirements
- Creates a new feature branch with numbered prefix (e.g., 001-user-auth)
- Generates a complete specification document following the standard template
- Validates the specification quality against checklist criteria
- Creates necessary directory structure and files

### `/sp.analyze` - Project Analysis
**Purpose**: Analyze the current project structure and dependencies
**Usage**: `/sp.analyze` or `/sp.analyze [optional focus area]`
**What it does**:
- Scans project directory structure and files
- Identifies technology stack and dependencies
- Analyzes version control state and branch patterns
- Reviews existing specifications and ongoing work
- Generates a comprehensive analysis report

### `/sp.constitution` - Project Constitution
**Purpose**: Create, update, or review the project constitution document
**Usage**: 
- `/sp.constitution` - Display current constitution
- `/sp.constitution new` - Create a new constitution
- `/sp.constitution update [changes]` - Update constitution with specific changes
- `/sp.constitution review` - Review constitution against current practices
**What it does**:
- Manages the project's core principles and governance rules
- Ensures consistency with development practices
- Tracks version and amendment history

### `/sp.plan` - Implementation Planning
**Purpose**: Create an implementation plan from a feature specification
**Usage**: `/sp.plan` or `/sp.plan [optional constraints or context]`
**What it does**:
- Reads the current feature specification
- Generates a multi-phase implementation approach
- Creates technical architecture recommendations
- Defines validation criteria for success criteria
- Organizes work by priority and dependencies

### `/sp.tasks` - Task Breakdown
**Purpose**: Create detailed implementation tasks from specification and plan
**Usage**: `/sp.tasks` or `/sp.tasks [optional focus area]`
**What it does**:
- Converts high-level plans into actionable tasks
- Creates unique identifiers for tracking
- Defines acceptance criteria for each task
- Establishes dependencies between tasks
- Maps tasks to functional requirements and success criteria

## How the Specify System Works

The Specify system follows a structured approach to feature development:

1. **Specification Phase**: Use `/sp.specify` to create clear, testable feature specifications
2. **Analysis Phase**: Use `/sp.analyze` to understand the current project state
3. **Planning Phase**: Use `/sp.plan` to create implementation strategies
4. **Tasking Phase**: Use `/sp.tasks` to break down work into manageable pieces
5. **Governance**: Use `/sp.constitution` to maintain project principles

## Best Practices

- Start with `/sp.specify` when you have a feature idea
- Use `/sp.analyze` when joining an existing project or before major changes
- Follow up `/sp.specify` with `/sp.plan` and `/sp.tasks` for implementation
- Review `/sp.constitution` periodically to ensure alignment with principles
- Each command creates appropriate files in the `specs/[feature-name]/` directory

## File Structure

The system creates and manages files in this structure:
- `.specify/` - System templates, scripts, and configuration
- `specs/[feature-name]/` - Individual feature specifications and plans
- `history/prompts/[feature-name]/` - AI interaction history (PHRs)
- `.specify/memory/constitution.md` - Project constitution

All commands work together to maintain consistency between specifications, plans, and implementation, ensuring clear traceability from requirements to code.