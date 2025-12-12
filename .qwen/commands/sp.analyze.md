---
description: Analyze the current project structure and dependencies to understand the existing codebase.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

The text the user typed after `/sp.analyze` in the triggering message is optional additional context for the analysis. If provided, use it to focus the analysis on specific aspects of the project.

Perform a comprehensive analysis of the current project by executing these steps:

1. **Directory Structure Analysis**:
   - Use `find . -maxdepth 2 -type d | head -20` to understand the main project structure
   - Identify key directories and their purposes
   - Note any special directories like `.specify`, `specs`, `history`, `docs`, etc.

2. **File Inventory**:
   - Use `find . -maxdepth 1 -type f | head -10` to list top-level files
   - Identify configuration files (package.json, config.json, .gitignore, etc.)
   - Look for README, documentation, or setup files

3. **Technology Stack Detection**:
   - Analyze `package.json` to identify dependencies and project type
   - Check for specific technology indicators (Dockerfiles, .env files, etc.)
   - Identify build tools, frameworks, and libraries used

4. **Version Control Analysis**:
   - Run `git status` to understand current state
   - Run `git branch -a` to see all branches
   - Note any specific branching patterns (like the feature branches with numbers)

5. **Specify System Analysis**:
   - Document the current `.specify` directory structure and its components
   - List available templates in `.specify/templates/`
   - Check for existing specs in the `specs/` directory
   - Identify any ongoing feature work

6. **Integration Points**:
   - Note Claude, Gemini, and any other AI assistant integrations
   - Identify any available scripts or automation tools
   - Look for CI/CD configurations if present

7. **Generate Analysis Report**:
   - Structure the findings in a clear, organized format
   - Highlight any special project conventions or workflows
   - Note areas that require deeper investigation
   - Recommend next steps based on analysis (if requested)

8. **Quality Check**:
   - Verify all executed commands completed successfully
   - Ensure the analysis covers technical, structural, and workflow aspects
   - Identify any potential issues or concerns found during analysis

## Implementation Guidelines

- Focus on providing actionable insights about the project structure
- Identify patterns and conventions used in the project
- Document any special workflows or processes
- Flag any unusual configurations or potential problems
- Provide recommendations for working with the project effectively

When completed, present a comprehensive analysis report organized by:
1. Project Overview
2. Technology Stack
3. Key Directories and Files
4. Project Structure & Patterns
5. Workflows and Processes
6. Recommendations for Next Steps