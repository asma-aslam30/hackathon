---
description: Create an implementation plan based on an existing feature specification.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.plan` provides additional context or constraints for the implementation plan. If empty, proceed with standard planning based on the current feature specification.

Follow these steps to create a comprehensive implementation plan:

1. **Identify Current Feature**:
   - Determine the current feature branch name using `get_current_branch` logic from common.sh
   - Locate the corresponding spec file in `specs/[branch-name]/spec.md`
   - Verify the spec file exists and is properly formatted

2. **Analyze Specification**:
   - Extract all user stories and priorities from the spec
   - Identify all functional requirements (FR-XXX items)
   - Extract success criteria (SC-XXX items)
   - Note any entities or data models defined
   - Identify edge cases and error scenarios
   - Check for any remaining [NEEDS CLARIFICATION] markers

3. **Generate Implementation Approach**:
   - Prioritize implementation tasks based on user story priorities (P1, P2, P3)
   - Map functional requirements to implementation tasks
   - Identify dependencies between different tasks/requirements
   - Consider technical architecture implications

4. **Create Plan Structure**:
   - Use the template from `.specify/templates/plan-template.md` or create standard structure
   - Organize plan by technical layers or user stories
   - Include both development tasks and validation steps

5. **Generate Detailed Plan** with the following sections:

   ### Technical Approach
   - Overall architecture approach
   - Key technical decisions to make
   - Technology stack recommendations
   - Data flow and component interactions

   ### Implementation Phases
   - **Phase 1** (P1 stories): Core functionality implementation
   - **Phase 2** (P2 stories): Enhanced functionality
   - **Phase 3** (P3 stories): Additional features
   - Each phase should be independently deliverable

   ### Development Tasks
   - Break down each functional requirement into specific development tasks
   - Estimate relative complexity for each task
   - Identify potential technical challenges or risks
   - Specify order of implementation based on dependencies

   ### Testing Strategy
   - Unit testing approach for each component
   - Integration testing for feature workflows
   - How to validate each success criterion
   - Edge case testing scenarios

   ### Validation Criteria
   - How to verify each functional requirement is met
   - How to measure success against success criteria
   - Definition of "done" for each phase

6. **Quality Assurance**:
   - Review plan against original specification
   - Ensure all requirements are addressed
   - Verify phases align with story priorities
   - Check that validation criteria match success criteria

7. **Save Plan**:
   - Save the plan to the appropriate feature directory: `specs/[branch-name]/plan.md`
   - Ensure file is properly formatted as Markdown

## Implementation Guidelines

- Keep implementation approach aligned with specification requirements
- Ensure phases are independently deliverable and testable
- Consider existing codebase when proposing technical approaches
- Balance technical best practices with delivery speed
- Plan for proper error handling and edge cases
- Include adequate testing at each phase
- Make sure validation criteria are measurable and specific

## Validation

After creating the plan, verify:
- All functional requirements from spec are addressed
- All success criteria have corresponding validation approaches
- Implementation phases align with user story priorities
- Technical approach is feasible with current technology stack
- Plan is realistic in terms of complexity and time
- All placeholders have been properly filled in

Report completion with the location of the generated plan file and a summary of the planned approach.