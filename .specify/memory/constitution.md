<!--
SYNC IMPACT REPORT
===================
Version Change: [INITIAL] → 1.0.0
Rationale: Initial constitution ratification for Embodied AI & Humanoid Systems Engineering book project

Modified Principles:
- PRINCIPLE_1: Spec-Driven Development
- PRINCIPLE_2: Academic Rigor & Reproducibility
- PRINCIPLE_3: Zero-Plagiarism & Source Attribution
- PRINCIPLE_4: AI-Driven Authoring Workflow
- PRINCIPLE_5: Docusaurus-First Architecture
- PRINCIPLE_6: Technical Precision & Evidence-Based Claims

Added Sections:
- Writing Standards
- Content Requirements
- Technical Constraints

Removed Sections: None (initial version)

Templates Requiring Updates:
✅ spec-template.md - aligned with academic requirements and source citation mandates
✅ plan-template.md - aligned with Docusaurus structure and constitution checks
✅ tasks-template.md - aligned with academic workflow and peer-review standards

Follow-up TODOs: None
-->

# Embodied AI & Humanoid Systems Engineering Book — Constitution

## Core Principles

### I. Spec-Driven Development

All book content, chapter structure, and technical documentation MUST originate from feature specifications created via `/sp.specify`. No content shall be generated without a corresponding spec defining scope, success criteria, and acceptance tests. Each chapter is treated as a "feature" with a testable specification that defines learning outcomes, content requirements, and validation criteria.

**Rationale**: Prevents scope creep, ensures traceability from user requirements to published content, and maintains consistency across the AI-driven authoring workflow.

### II. Academic Rigor & Reproducibility

Every technical claim, architecture decision, and engineering principle MUST be supported by peer-reviewed sources, official technical documentation, or reproducible experimental results. Minimum 15 credible sources per major chapter (5,000–7,000 words). All sources MUST use APA citation format with full bibliographic entries.

**Rationale**: Maintains academic credibility, ensures claims are verifiable, and provides readers with paths to deeper investigation. Aligns with university-level technical writing standards.

### III. Zero-Plagiarism & Source Attribution

All content MUST achieve 0% plagiarism as measured by academic plagiarism detection tools. Direct quotes require quotation marks and inline citations. Paraphrased content requires citation. AI-generated content MUST be fact-checked against primary sources and properly attributed. No "common knowledge" exceptions—cite everything.

**Rationale**: Protects intellectual property, maintains ethical standards, ensures legal compliance, and builds trust with academic and professional audiences.

### IV. AI-Driven Authoring Workflow

All content generation MUST use Claude Code with Spec-Kit Plus workflow: `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement`. Each chapter follows the red-green-refactor cycle: write acceptance tests (learning objectives) first, generate draft content, validate against sources, refactor for clarity. Human review gates MUST occur after planning and before final publication.

**Rationale**: Ensures systematic development, maintains quality through automated checks, enables version control of authoring decisions, and creates auditable development history through Prompt History Records (PHRs).

### V. Docusaurus-First Architecture

All content MUST be authored in Docusaurus-compatible Markdown with YAML frontmatter. Navigation structure defined via `sidebars.js`. GitHub Pages deployment MUST be validated before publication. PDF export MUST preserve citations, cross-references, and formatting. All diagrams MUST be SVG or Mermaid for version control and readability.

**Rationale**: Provides static site generation for web deployment, enables collaborative editing via Git, ensures multi-format output (web + PDF), and maintains professional presentation standards.

### VI. Technical Precision & Evidence-Based Claims

No marketing language, vague generalizations, or unsubstantiated claims. Every architecture decision MUST include trade-off analysis. Every algorithm MUST include computational complexity. Every system design MUST specify failure modes and mitigation strategies. Comparative claims require quantitative evidence from benchmarks or published studies.

**Rationale**: Distinguishes academic/technical writing from promotional content, enables informed decision-making, and provides practical value to engineering practitioners.

## Writing Standards

### Voice & Tone

- Concise, professional, and technical
- Third-person academic perspective (avoid "I", "we", "you" except in examples)
- Active voice preferred for clarity
- No superlatives without quantitative support (e.g., "fastest" requires benchmark data)
- Industry-standard terminology with definitions for domain-specific terms

### Formatting Requirements

- APA 7th edition for all citations
- Harvard referencing for in-text citations: (Author, Year)
- Bibliography section at end of each chapter
- Code blocks MUST specify language for syntax highlighting
- Technical terms italicized on first use with definition
- Equations formatted in LaTeX/KaTeX notation

### Source Quality Standards

**Acceptable Sources**:
- Peer-reviewed journal articles (IEEE, ACM, Springer, Nature, Science)
- Conference proceedings from top-tier venues (NeurIPS, ICRA, CoRL, RSS)
- Official technical documentation from authoritative organizations (ISO, IEEE standards)
- Government/military technical reports (DARPA, DoD, NIST)
- Manufacturer datasheets and technical specifications
- Open-source project documentation (if maintained by recognized institutions)

**Unacceptable Sources**:
- Blog posts (unless from recognized technical experts with peer-reviewed work)
- Wikipedia (may use for background, must verify with primary sources)
- Marketing materials or whitepapers without independent validation
- Social media posts
- Unattributed web content
- AI-generated content without source validation

## Content Requirements

### Chapter Structure

Each chapter MUST include:

1. **Learning Objectives**: 3–5 measurable outcomes (e.g., "Explain the trade-offs between model-based and model-free reinforcement learning for bipedal locomotion")
2. **Technical Context**: Problem statement, historical development, current state-of-the-art
3. **Core Content**: Theory, implementation details, worked examples
4. **Architecture Decisions**: When alternatives exist, explicit trade-off analysis with ADR references
5. **Practical Examples**: Runnable code snippets, configuration files, or simulation parameters
6. **Validation Criteria**: How readers can verify their understanding (problems, experiments)
7. **Bibliography**: Minimum 15 credible sources in APA format

### Code Examples

- MUST be syntactically correct and tested
- MUST include dependency versions (e.g., `numpy==1.24.3`)
- MUST specify runtime environment (Python 3.11, ROS2 Humble, etc.)
- MUST include expected output or behavior
- SHOULD be self-contained (runnable without external files)

### Diagrams & Visualizations

- Architecture diagrams using Mermaid or draw.io (exported as SVG)
- Mathematical proofs using KaTeX/LaTeX
- Data flow diagrams following UML or standard notation
- System diagrams with labeled interfaces and data flows
- All diagrams MUST have captions and reference numbers

## Technical Constraints

### Word Count & Scope

- Target: 5,000–7,000 words per major chapter
- Minimum: 15 credible sources per chapter
- Code examples: 500–1,000 lines per chapter (well-commented)
- Diagrams: 3–5 per chapter minimum

### Deployment Requirements

- MUST build successfully with `npm run build` (Docusaurus)
- MUST deploy to GitHub Pages without errors
- MUST render correctly on desktop and mobile browsers
- MUST generate PDF with intact citations and cross-references
- MUST pass accessibility checks (WCAG 2.1 AA minimum)

### Performance Standards

- Page load time: <3 seconds on 3G connection
- Search functionality: <500ms response time
- Build time: <5 minutes for full site
- PDF generation: <2 minutes per chapter

### Version Control Standards

- All content changes MUST be committed with descriptive messages
- Branch naming: `feature/<chapter-number>-<chapter-slug>`
- Pull requests MUST include Prompt History Record (PHR) reference
- Architecture Decision Records (ADRs) MUST be created for structural changes

## Governance

### Amendment Process

1. Proposed changes MUST be documented in Prompt History Record (PHR)
2. Architectural changes MUST create Architecture Decision Record (ADR) via `/sp.adr`
3. Version bump rules:
   - **MAJOR**: Removal of core principles, incompatible workflow changes
   - **MINOR**: New principles added, expanded guidance, new content categories
   - **PATCH**: Clarifications, typo fixes, formatting improvements
4. All amendments require validation that dependent templates remain consistent

### Compliance Verification

- Every `/sp.plan` output MUST include "Constitution Check" section
- Every `/sp.tasks` output MUST verify test requirements align with academic standards
- Pre-commit hooks SHOULD check for placeholder tokens and citation format
- Pull request reviews MUST verify source quality and plagiarism checks

### Complexity Justification

Any violation of simplicity principles (e.g., introducing new dependencies, adding non-standard Docusaurus plugins) MUST be justified in the Implementation Plan with:
- **Why Needed**: Specific problem being solved
- **Simpler Alternative Rejected Because**: Concrete technical reason
- **Exit Strategy**: How to remove if requirements change

### Documentation Standards

Use `.specify/memory/constitution.md` (this file) as the authoritative governance document. Runtime development guidance for agents and developers should reference this constitution but not duplicate its content. All architectural decisions referencing constitution principles MUST cite specific principle numbers (I–VI).

---

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
