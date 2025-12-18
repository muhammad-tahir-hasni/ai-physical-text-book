---
id: 001
title: Course Book Specification
stage: spec
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-course
branch: 001-physical-ai-course
user: apple
command: /sp.specify
labels: ["spec", "embodied-ai", "ros2", "isaac", "vla", "course-book", "docusaurus"]
links:
  spec: specs/001-physical-ai-course/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-course/spec.md
  - specs/001-physical-ai-course/checklists/requirements.md
tests:
  - Validation checklist: All 12 items passed
---

## Prompt

```text
Embodied AI & Humanoid Systems Engineering – Complete 4-Module Course Book

Purpose:
Generate a complete, structured, academically credible course-book covering all four official modules of "Physical AI & Humanoid Robotics." The book must help students understand how digital AI systems extend into the physical world and how embodied intelligence is implemented using ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action (VLA) pipelines. The output should read like a polished academic primer suitable for beginner-to-intermediate AI/robotics students preparing for a Physical AI capstone project.

Scope:
- Include ONLY the four modules specified below.
- Focus on conceptual architecture, data flow, and the connection between simulation and real-world humanoid robotics.
- Include clear **textual descriptions of diagrams**, tables, and stepwise conceptual examples (no code implementation).
- Exclude hardware setup, pricing, lab instructions, cloud solutions, and ethics/history digressions.

Modules:

1. Module 1 — The Robotic Nervous System (ROS 2)
   - ROS 2 architecture: nodes, topics, services, actions, parameters
   - rclpy Python bridge to robot controllers
   - URDF fundamentals for humanoid robots
   - Conceptual example: sensor → ROS → actuator pipeline
   - Textual diagram description: show a sensor connected to ROS 2 nodes, publishing topics to controllers, leading to actuators.

2. Module 2 — The Digital Twin (Gazebo & Unity)
   - Physics simulation: gravity, collisions, rigid body dynamics
   - High-fidelity environment creation in Unity
   - Sensor simulation: LiDAR, depth cameras, IMUs
   - Workflow textual diagrams: robot model imported → simulated in Gazebo → sensor data generated → visualized in Unity.

3. Module 3 — The AI-Robot Brain (NVIDIA Isaac)
   - Isaac Sim: photorealistic simulation, synthetic data generation
   - Isaac ROS: VSLAM, navigation, perception
   - Nav2: path planning for bipedal humanoid robots
   - Textual architecture diagrams: Isaac Sim environment → ROS 2 nodes → SLAM module → navigation planner → actuators.

4. Module 4 — Vision-Language-Action (VLA)
   - Whisper for voice-to-action processing
   - LLM-based cognitive planning for natural language tasks
   - Mapping natural language to ROS 2 action sequences
   - Capstone: Autonomous Humanoid pipeline (voice command → plan → navigation → perception → manipulation)
   - End-to-end textual diagrams showing multi-modal data flow from voice, vision, LLM planning, ROS 2 action execution, to robot actuators.

Required Book Outcomes:
- Explain Physical AI and Embodied Intelligence clearly
- Demonstrate simulation + real hardware conceptual connections
- Show how humanoid robots understand, plan, and act
- Teach all four modules with clarity, diagrams as textual descriptions, and conceptual examples
- Include real educational context for humanoid robotics relevance
- Prepare students for the capstone project

Writing Standards:
- Academic tone, but accessible for motivated students
- Structured chapters with smooth logical flow
- Use Markdown format
- No coding required, focus on conceptual architecture only
- 3,000–5,000 words
- Include APA citations if external claims are made

Success Criteria:
- Each module is a self-contained chapter
- Students can understand sensor → AI → action pipeline
- Capstone pipeline conceptually complete and actionable
- Book reads like a polished course primer suitable for hackathon evaluation
- All diagrams presented as clear textual descriptions to convey architecture visually
```

## Response snapshot

Created comprehensive feature specification for Physical AI & Humanoid Systems Engineering course book with four prioritized user stories:

**User Story 1 (P1)**: ROS 2 Fundamentals Comprehension - Foundation for understanding sensor-actuator coordination through ROS 2 architecture (nodes, topics, services, actions). Critical prerequisite for all subsequent modules.

**User Story 2 (P2)**: Simulation Environment Understanding - Digital twins in Gazebo/Unity for safe algorithm testing before hardware deployment. Covers physics simulation and sensor data generation workflows.

**User Story 3 (P3)**: NVIDIA Isaac Integration - Bridges simulation and AI with Isaac Sim photorealistic environments, Isaac ROS VSLAM/navigation, and Nav2 path planning for bipedal humanoids.

**User Story 4 (P4)**: Vision-Language-Action Pipeline - End-to-end capstone integration showing voice commands → LLM planning → ROS 2 actions → robot execution with failure modes.

**24 Functional Requirements** covering:
- Complete ROS 2 architecture explanations (FR-001 to FR-003)
- Physics and sensor simulation (FR-004 to FR-007)
- Isaac Sim/ROS integration (FR-008 to FR-010)
- VLA pipeline components (FR-011 to FR-013)
- Content quality standards (FR-014 to FR-024): textual diagrams, academic tone, 3,000-5,000 words, APA citations, Markdown/Docusaurus compatibility

**14 Success Criteria** with measurable outcomes:
- 90% accuracy on concept checks (SC-001)
- Complete workflow descriptions (SC-002, SC-003, SC-004)
- Zero implementation details (SC-008)
- 3,000-5,000 words with balanced coverage (SC-011)
- 0% plagiarism with APA citations (SC-012)
- Docusaurus-ready formatting (SC-013)

**6 Edge Cases** identified: Prior knowledge gaps, URDF model variations, sim-to-real physics divergence, hardware constraints, infeasible LLM plans, ambiguous voice commands.

**8 Assumptions** documented including student background (basic programming, no robotics required), target audience (undergrad/grad students), content delivery (Docusaurus/PDF), and focus on conceptual understanding over implementation.

**Comprehensive Out of Scope** section excludes hardware setup, pricing, lab procedures, cloud deployment, ethics, history, code implementation, mathematical derivations, RTOS, platform-specific instructions, and advanced topics.

Specification validated against quality checklist: ✅ All 12 validation criteria passed with zero [NEEDS CLARIFICATION] markers, testable requirements, measurable success criteria, and clear scope boundaries.

## Outcome

- ✅ Impact: Complete feature specification ready for planning phase. Establishes clear learning objectives, content requirements, and quality standards for four-module course book on embodied AI and humanoid robotics.
- 🧪 Tests: Validation checklist passed (12/12 criteria) - no blocking issues identified
- 📁 Files: Created spec.md (182 lines) and checklists/requirements.md (validation results)
- 🔁 Next prompts: Run `/sp.plan` to develop implementation architecture for book structure, textual diagram templates, and content organization
- 🧠 Reflection: Specification successfully balances academic rigor with accessibility for beginner-to-intermediate students. Strong alignment between user stories (P1→P4 progression), functional requirements (24 testable criteria), and success criteria (14 measurable outcomes). Textual diagram descriptions solve visualization challenge while maintaining conceptual focus. Zero clarifications needed due to comprehensive user input defining all four modules with explicit scope boundaries.

## Evaluation notes (flywheel)

- Failure modes observed: None - user input was exceptionally detailed with explicit module breakdown, scope constraints, and success criteria
- Graders run and results (PASS/FAIL): PASS - Validation checklist: 12/12 criteria met (content quality 4/4, requirement completeness 8/8, feature readiness 4/4)
- Prompt variant (if applicable): Standard /sp.specify workflow with quality validation checklist generation
- Next experiment (smallest change to try): During planning phase, validate that textual diagram description templates can convey complex multi-stage pipelines (ROS communication, Isaac SLAM, VLA end-to-end) without visual rendering
