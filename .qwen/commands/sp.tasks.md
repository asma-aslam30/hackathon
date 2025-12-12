---
description: Create a detailed breakdown of implementation tasks from a specification and plan.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.tasks` provides additional context or specific focus areas for the task breakdown. If empty, proceed with standard task breakdown based on existing specification and plan.

Follow these steps to create a comprehensive task breakdown:

1. **Identify Current Feature**:
   - Determine the current feature branch name
   - Locate the corresponding spec file in `specs/[branch-name]/spec.md`
   - Locate the plan file if it exists in `specs/[branch-name]/plan.md`
   - Verify both files exist and are properly formatted

2. **Analyze Existing Documents**:
   - If plan exists: extract technical approach, phases, and validation criteria
   - If no plan exists: derive technical approach from spec alone
   - Extract all user stories and their priorities (P1, P2, P3)
   - Extract all functional requirements (FR-XXX items)
   - Identify success criteria (SC-XXX items) that need implementation
   - Note any data models or key entities defined in the spec

3. **Generate Task Categories**:
   - Setup/Configuration tasks
   - Core functionality tasks
   - Enhancement tasks
   - Integration tasks
   - Testing tasks
   - Documentation tasks

4. **Create Detailed Task Breakdown** with the following structure:

   # Task Breakdown for [Feature Name]

   **Feature**: `[branch-name]`
   **Created**: [Current date]
   **Status**: Planned

   ## Task Categories

   ### 1. Setup Tasks
   - **Task ID**: SETUP-001
   - **Description**: [Detailed description of setup task]
   - **Priority**: [P1/P2/P3 based on feature priority]
   - **Dependencies**: [Any dependencies on other tasks or external factors]
   - **Acceptance Criteria**: [How to verify this task is complete]

   ### 2. Core Implementation Tasks
   - **Task ID**: CORE-001
   - **Description**: [Detailed description of implementation task]
   - **Priority**: [P1/P2/P3 based on feature priority]
   - **Dependencies**: [Any dependencies on other tasks or external factors]
   - **Acceptance Criteria**: [How to verify this task is complete]
   - **Related FRs**: [Which functional requirements this task addresses]

   ### 3. Testing Tasks
   - **Task ID**: TEST-001
   - **Description**: [Detailed description of testing task]
   - **Priority**: [P1/P2/P3 based on associated feature priority]
   - **Dependencies**: [Any dependencies on other tasks or external factors]
   - **Acceptance Criteria**: [How to verify this task is complete]
   - **Related SCs**: [Which success criteria this test validates]

   [Continue for other categories as needed]

5. **Task Prioritization**:
   - Ensure P1 feature tasks are prioritized in early task sequences
   - Mark clear dependencies between tasks
   - Identify parallelizable tasks where possible
   - Group related tasks for efficient implementation

6. **Validation and Quality Checks**:
   - Ensure every functional requirement has at least one corresponding task
   - Verify that success criteria validation is included in testing tasks
   - Check that all user stories are addressed through the task breakdown
   - Confirm that task dependencies are logically sound

7. **Save Task Breakdown**:
   - Save to the appropriate feature directory: `specs/[branch-name]/tasks.md`
   - Ensure file is properly formatted as Markdown with clear structure

## Implementation Guidelines

- Create specific, actionable tasks that can be assigned and tracked
- Ensure tasks are of manageable size (not too granular, not too broad)
- Include both development tasks and validation tasks
- Link tasks back to specific functional requirements and success criteria
- Consider the team's technical capabilities and constraints
- Plan for proper testing and validation of each implementation
- Design tasks to enable iterative and incremental development

## Task Format Requirements

Each task should include:
- Unique identifier (e.g., SETUP-001, CORE-001, TEST-001)
- Clear, specific description of what needs to be done
- Priority level (P1/P2/P3) aligned with feature priorities
- Dependencies on other tasks or external factors
- Clear acceptance criteria for completion verification
- References to related functional requirements or success criteria

## Validation

After creating the task breakdown, verify:
- Every functional requirement (FR-XXX) maps to at least one task
- Success criteria (SC-XXX) map to appropriate testing/validation tasks
- All user stories are addressed through the task breakdown
- Task dependencies are properly defined
- Task priorities align with feature priorities
- Tasks are of appropriate granularity for effective tracking
- All placeholders have been properly filled in

Report completion with the location of the generated tasks file and a summary of the task categories and total task count.