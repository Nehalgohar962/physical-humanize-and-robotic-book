<!--
Sync Impact Report:
Version change: 0.0.0 (initial) → 1.0.0
List of modified principles:
  - Initial principles defined: Accuracy, Clarity, Reproducibility, Rigor, Modularity
Added sections:
  - Key Standards
  - Constraints and Success Criteria
Removed sections:
  - None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .gemini/commands/sp.adr.toml: ⚠ pending
  - .gemini/commands/sp.analyze.toml: ⚠ pending
  - .gemini/commands/sp.checklist.toml: ⚠ pending
  - .gemini/commands/sp.clarify.toml: ⚠ pending
  - .gemini/commands/sp.constitution.toml: ✅ updated
  - .gemini/commands/sp.git.commit_pr.toml: ⚠ pending
  - .gemini/commands/sp.implement.toml: ⚠ pending
  - .gemini/commands/sp.phr.toml: ⚠ pending
  - .gemini/commands/sp.plan.toml: ⚠ pending
  - .gemini/commands/sp.specify.toml: ⚠ pending
  - .gemini/commands/sp.tasks.toml: ⚠ pending
  - README.md: ⚠ pending
Follow-up TODOs:
  - None
-->
# Humanoid Robotics & Physical AI – AI/Spec-Driven Book Creation Constitution

## Core Principles

### I. Accuracy
All technical explanations, ROS 2 code snippets, and AI/LLM references must be verified against authoritative sources and documentation.

### II. Clarity
Content must be understandable by students and developers with basic computer science and robotics background.

### III. Reproducibility
All code, simulations, and experiments must be runnable; instructions must allow replication of results.

### IV. Rigor
Use up-to-date robotics research, peer-reviewed papers, and reliable technical references wherever possible.

### V. Modularity
Content structured in modules (e.g., Module 1–Module 4) with clear separation of specifications, plans, and tasks.

## Key Standards

- All factual and technical claims must be traceable to official documentation or peer-reviewed sources.
- Citation format: IEEE or APA style for robotics and AI references.
- Source types: Minimum 40% peer-reviewed academic articles, 30% official robotics/AI documentation, 30% verified code tutorials.
- Plagiarism check: 0% tolerance; all content must be original or properly cited.
- Writing clarity: Flesch-Kincaid grade 10-12 for readability.
- Code examples: Must be syntactically correct and compatible with current ROS 2 and Python/JavaScript versions.

## Constraints and Success Criteria

### Constraints
- Word count per module: 2,000–3,000 words
- Overall book length: 8,000–12,000 words
- Format: Markdown compatible with Docusaurus + runnable code snippets
- Deployment: GitHub Pages via Docusaurus
- Timeline: Complete all modules and deployment within project schedule

### Success Criteria
- All modules verified to run successfully on local system
- Citations and references traceable and correct
- Zero plagiarism detected
- Passes internal review for technical accuracy and clarity
- Book deployed on GitHub Pages and accessible publicly

## Governance

### Amendment Procedure
Amendments to this constitution must follow the established ADR (Architectural Decision Record) process, requiring consensus from project stakeholders and approval from the lead architect(s). Minor clarifications or typo fixes may be proposed via pull request and reviewed by maintainers.

### Versioning Policy
This constitution uses semantic versioning (MAJOR.MINOR.PATCH). MAJOR for backward incompatible changes, MINOR for new sections/principles, PATCH for clarifications/typos.

### Compliance Review Expectations
Compliance with this constitution will be periodically reviewed (e.g., quarterly) by the project lead(s) to ensure ongoing adherence, especially for new features or major refactorings. Audits will assess alignment with principles, standards, and constraints.

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08