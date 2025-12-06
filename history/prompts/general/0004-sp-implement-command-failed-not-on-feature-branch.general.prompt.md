---
id: 0004
title: "/sp.implement" command failed - Not on feature branch
stage: general
date: 2025-12-06
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: unknown
command: /sp.phr
labels: ["sp.implement", "failure", "general"]
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

--title "/sp.implement" command failed - Not on feature branch --stage general

## Response snapshot

The `/sp.implement` command failed because the current branch is not a feature branch. PHR recorded for this event.

## Outcome

- ✅ Impact: Recorded the failure of `/sp.implement` command due to incorrect branch.
- 🧪 Tests: none
- 📁 Files: none
- 🔁 Next prompts: User needs to switch to a feature branch before running `/sp.implement`.
- 🧠 Reflection: Ensure proper branch context before executing feature-specific commands.

## Evaluation notes (flywheel)

- Failure modes observed: `/sp.implement` command requires a feature branch, but was run on 'main'.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add a pre-check in `/sp.implement` to validate branch naming convention.
