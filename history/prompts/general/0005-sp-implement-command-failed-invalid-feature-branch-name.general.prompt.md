---
id: 0005
title: "/sp.implement" command failed - Invalid feature branch name
stage: general
date: 2025-12-06
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: feature/my-task
user: unknown
command: /sp.phr
labels: ["sp.implement", "failure", "general", "branch-naming"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - none
tests:
 - none
---

## Prompt

--title "/sp.implement" command failed - Invalid feature branch name --stage general

## Response snapshot

The `/sp.implement` command failed because the branch name `feature/my-task` does not follow the required pattern `001-feature-name`. PHR recorded for this event.

## Outcome

- ✅ Impact: Recorded the failure of `/sp.implement` command due to an invalid branch name.
- 🧪 Tests: none
- 📁 Files: none
- 🔁 Next prompts: User needs to rename or switch to a feature branch with a valid naming convention (e.g., `001-my-task`) before running `/sp.implement`.
- 🧠 Reflection: Enforce strict branch naming conventions earlier in the workflow for commands that depend on it.

## Evaluation notes (flywheel)

- Failure modes observed: `/sp.implement` command failed due to non-compliant branch name `feature/my-task`.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Integrate branch naming validation into pre-commit hooks or the `/sp.tasks` command.
