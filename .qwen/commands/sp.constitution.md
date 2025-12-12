---
description: Create, update, or review the project constitution document that defines core principles and governance.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.constitution` specifies what action to take with the project constitution:

- If empty: Display the current constitution from `.specify/memory/constitution.md`
- If "new" or "create": Create a new constitution based on the template
- If "update" or "amend": Update the current constitution with specific changes
- If "review": Review and validate the current constitution against project practices
- Otherwise: Treat as specific content to incorporate into the constitution

## Actions

### Action A: Display Current Constitution
- Read and display the current constitution from `.specify/memory/constitution.md`
- Highlight key sections for quick reference
- Note the version, ratification date, and last amendment date

### Action B: Create New Constitution
1. Load the constitution template from `.specify/templates/constitition.md` (or create from standard structure)
2. Customize for the specific project:
   - Replace [PROJECT_NAME] with the actual project name
   - Define appropriate core principles based on project type
   - Set project-specific governance rules
3. Save to `.specify/memory/constitution.md`
4. Update version information and dates

### Action C: Update Constitution
1. Load the current constitution
2. Apply requested amendments while preserving existing structure
3. Validate that amendments follow constitutional amendment procedures
4. Update version, amendment date, and change log
5. Save the updated constitution

### Action D: Review Constitution
1. Compare current constitution with actual project practices
2. Identify any inconsistencies between constitution and practice
3. Suggest amendments if needed to align constitution with reality
4. Verify all principles are still relevant and applicable
5. Check for any missing principles that should be added

## Constitution Template Structure

The constitution should follow this structure:

```
# [PROJECT_NAME] Constitution
<!-- Example: Spec Constitution, TaskFlow Constitution, etc. -->

## Core Principles

### [PRINCIPLE_1_NAME]
[PRINCIPLE_1_DESCRIPTION]

### [PRINCIPLE_2_NAME]
[PRINCIPLE_2_DESCRIPTION]

### [PRINCIPLE_3_NAME]
[PRINCIPLE_3_DESCRIPTION]

## [SECTION_2_NAME]
[SECTION_2_CONTENT]

## [SECTION_3_NAME]
[SECTION_3_CONTENT]

## Governance
[GOVERNANCE_RULES]

**Version**: [CONSTITUTION_VERSION] | **Ratified**: [RATIFICATION_DATE] | **Last Amended**: [LAST_AMENDED_DATE]
```

## Implementation Guidelines

- Ensure all constitutional amendments follow proper procedures (document the process)
- Maintain consistency with other project governance documents
- Align with organization's broader standards and practices
- Consider impact on existing processes and workflows
- Ensure constitution remains practical and enforceable
- Track constitutional changes and their justifications

## Validation

After any constitution operation, always verify:
- All placeholder values have been replaced with actual information
- Version information is current and correctly formatted
- Amendment procedures were followed if changes were made
- Constitution remains consistent with other project documents
- All principles are actionable and measurable where appropriate

Report the outcome of the requested operation with specific details about what was done and any important considerations for future reference.