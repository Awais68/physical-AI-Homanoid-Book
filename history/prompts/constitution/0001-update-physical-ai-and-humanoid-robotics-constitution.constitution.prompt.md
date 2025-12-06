---
id: 0001
title: Update Physical AI and Humanoid Robotics Constitution
stage: constitution
date: 2025-12-03
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: system
command: /sp.constitution
labels: ["constitution", "update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 N/A
---

## Prompt

Topic: Physical AI and Humanoid Robotics
1. Purpose of This Constitution

This constitution defines the rules, structure, quality standards, and generation principles required to produce a complete, high-quality learning resource book on Physical AI and Humanoid Robotics.
The book should be thorough, academically reliable, technically accurate, accessible for intermediate learners, and engaging for beginners.

2. Target Audience

Undergraduate engineering & CS students

Robotics, AI, and Mechatronics learners

Self-learners transitioning into humanoid robotics

Early-career developers exploring embodied AI

The book must balance technical depth with clear explanations.

3. Writing Style Requirements

The book must follow these writing rules:

3.1 Tone & Language

Formal academic writing

Clear, concise, and logically structured

No slang or informal expressions

Use precise engineering terminology

Maintain consistency across all chapters

3.2 Explanatory Requirements

Each concept should include:

A definition

A simple explanation

A deeper technical explanation

A real-world example

Diagrams or figure descriptions

Applications and relevance

3.3 AI-Generated Content Quality

No hallucinations

No invented facts, technologies, or research

Only real concepts, verified principles, and validated engineering practices

Cite well-known robotics research accurately when needed

4. Book Architecture

The book must contain the following structure:

4.1 Front Matter

Title Page

Preface

How to Use this Book

Learning Path Overview

Glossary of Key Terms

4.2 Core Chapters

Each chapter must follow a uniform structure:

Chapter Structure Template

Introduction

Key Concepts and Definitions

Technical Foundations

Real-World Implementations

Case Studies

Diagrams, Models, and System Architecture

Hands-On Exercises or Experiments

Common Mistakes & Misconceptions

Summary

Additional Reading

Required Chapters

Foundations of Physical AI

Introduction to Humanoid Robotics

Embodied Intelligence & Cognitive Robotics

Robot Anatomy

Actuators, Motors & Artificial Muscles

Sensors & Perception Systems

Locomotion in Humanoids

Manipulation & Grasping

Robot Control Systems

Physics-Driven Motion Planning

Reinforcement Learning for Embodied Agents

Human-Robot Interaction

Safety, Ethics, and Alignment

Industrial, Medical & Social Applications

Future of Physical AI and Super-Humanoids

4.3 Back Matter

Appendix

Mathematical Foundations

Code Samples / Pseudocode

Bibliography (real, reputable sources only)

Index

5. Technical Accuracy Rules

The book must comply with the following standards:

5.1 Robotics & AI Accuracy

Use real robotics terminology (e.g., torque control, proprioception, inverse kinematics).

Avoid fictional technologies unless clearly described as hypothetical.

Provide proper physical models (e.g., rigid body dynamics).

5.2 Mathematical Requirements

Where necessary, include:

Equations

Derivations

Graphs

Algorithmic flowcharts

All math must be correct and verifiable.

5.3 Engineering Standards

When describing engineering systems, follow:

ISO robotics guidelines

Established research papers

Industry practices from Boston Dynamics, Tesla, Agility Robotics, etc.

6. Visual & Diagram Guidelines

AI must generate:

Clear diagram descriptions

Labels: inputs, outputs, motors, joints, sensors, controllers

Flowcharts for control architectures

Modular block diagrams (ROS-based or classical robotics)

No purely decorative or meaningless diagrams are allowed.

7. Learning Science Principles

The book must integrate:

Scaffolding (simple to complex)

Knowledge reinforcement

Hands-on exercises

Real-world projects

Review checkpoints

Short quizzes after each chapter are required.

8. Ethics & Safety Guidelines

Content must:

Promote safe experimentation

Educate about ethical concerns

Avoid instructions that enable harmful robot misuse

Emphasize aligned robotics and safe control systems

9. Output Requirements

Whenever the model generates chapters or sections, the output must follow:

High-level clarity

Strict adherence to chapter template

Depth suitable for academic teaching

Structured formatting: headings, subheadings, bullets, diagrams

10. Consistency Rules

The entire book must follow:

Unified terminology

Stable writing voice

Consistent formatting

Comparable depth across chapters

No chapter should feel weaker or missing content.

11. Generation Constraints

The model must:

Avoid self-reference

Avoid content repetition

Ensure every section adds new value

Generate logically complete chapter segments

12. Evaluation Criteria

Every generated chapter or section is considered high-quality only if it meets:

Technical correctness

Clarity

Completeness

Educational value

Alignment with this constitution

Final Statement

This constitution acts as the governing framework for generating a comprehensive, accurate, and professional-grade learning resource book on Physical AI and Humanoid Robotics. All future content generated must strictly follow these rules.

## Response snapshot

The constitution for "Physical AI and Humanoid Robotics" has been updated to version 1.0.0, ratified and last amended on 2025-12-03.
The update involved replacing the generic template with the detailed book constitution provided in the user's input.
The following templates were reviewed and marked as updated for consistency:
- .specify/templates/plan-template.md
- .specify/templates/spec-template.md
- .specify/templates/tasks-template.md
- .specify/templates/commands/*.md (conceptual check)

Suggested commit message:
docs: add constitution for Physical AI and Humanoid Robotics v1.0.0

## Outcome

- ✅ Impact: Constitution updated with new content for Physical AI and Humanoid Robotics book.
- 🧪 Tests: No tests executed as part of constitution update.
- 📁 Files: Modified .specify/memory/constitution.md.
- 🔁 Next prompts: N/A
- 🧠 Reflection: Successfully updated the constitution based on user input, including versioning and consistency checks.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
