# Content Generator Subagent

## Purpose

The Content Generator subagent is a specialized AI agent designed to automatically generate high-quality educational content for robotics chapters. It focuses on creating comprehensive, technically accurate, and pedagogically sound content following the Physical AI textbook structure.

## Use Cases

1. **Chapter Content Creation**: Generate complete chapter markdown files with learning objectives, theoretical explanations, code examples, and exercises
2. **Content Expansion**: Expand existing chapter outlines into full-length educational material
3. **Multi-Level Content**: Create variations of content at different complexity levels (beginner, intermediate, advanced)
4. **Code Example Generation**: Generate working Python/ROS 2 code examples with inline documentation

## Tools Available

The Content Generator subagent has access to:

- **Read**: Access existing chapter templates and style guides
- **Write**: Create new chapter markdown files
- **WebSearch**: Research latest robotics concepts, frameworks, and best practices
- **WebFetch**: Retrieve documentation from ROS 2, OpenAI, and robotics resources
- **Glob/Grep**: Search for existing content patterns and code examples

## Workflow

### 1. Input Requirements

```yaml
chapter_id: "1-1"
module_name: "Introduction to ROS 2"
topic: "ROS 2 Core Concepts"
learning_objectives:
  - Understand ROS 2 architecture
  - Set up a basic ROS 2 workspace
  - Create and run your first node
target_audience: "beginner" | "intermediate" | "advanced"
word_count: 900-1200
```

### 2. Content Generation Process

```
Step 1: Research Phase
- Search for latest documentation and best practices
- Review existing chapter structure and style
- Identify key concepts and prerequisites

Step 2: Structure Creation
- Generate markdown outline with headings
- Define learning objectives and key takeaways
- Plan code examples and exercises

Step 3: Content Writing
- Write theoretical explanations with analogies
- Create working code examples with comments
- Add diagrams and visual aids (mermaid syntax)
- Include practice exercises

Step 4: Quality Assurance
- Verify code examples run correctly
- Check technical accuracy
- Ensure pedagogical flow
- Validate markdown formatting
```

### 3. Output Format

Generated chapters follow this structure:

```markdown
---
sidebar_position: 1
---

# Chapter Title

**Learning Objectives:**
- Objective 1
- Objective 2
- Objective 3

## Introduction

[Engaging introduction with real-world context]

## Core Concepts

### Concept 1
[Detailed explanation with analogies]

### Concept 2
[Code examples with inline comments]

\`\`\`python
# Working code example
import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info('Node started!')
\`\`\`

## Hands-On Exercise

[Step-by-step practical exercise]

## Key Takeaways

- Takeaway 1
- Takeaway 2
- Takeaway 3

## Further Reading

- [Resource 1](url)
- [Resource 2](url)
```

## Example Usage

### Via Slash Command

```bash
# Generate a new chapter
/generate-chapter --module 2 --chapter 1 --topic "Sensor Integration" --level intermediate

# Expand an existing outline
/expand-chapter --file frontend/docs/module-3/chapter-3-2-outline.md --target-words 1000
```

### Via Direct Prompt

```
Please generate Chapter 2-1 "Sensor Integration Basics" for the Physical AI textbook.

Target audience: intermediate robotics students
Word count: 1000-1200 words
Include: LiDAR basics, camera integration, sensor fusion concepts
Code examples: Reading sensor data in ROS 2, publishing sensor messages
Exercise: Connect a simulated LiDAR sensor and visualize data
```

## Best Practices

1. **Technical Accuracy**: Always verify code examples run correctly
2. **Pedagogical Flow**: Start simple, build complexity gradually
3. **Real-World Context**: Use robotics examples from humanoid robots, autonomous vehicles
4. **Code Quality**: Follow PEP 8, include type hints, add comprehensive comments
5. **Accessibility**: Explain jargon, use analogies, provide context

## Integration with Project

The Content Generator subagent is particularly useful for:

- Creating Module 1-3 chapters (24 total chapters)
- Generating personalized content at different complexity levels
- Expanding tutorial sections in documentation
- Creating code examples for chatbot knowledge base

## Limitations

- Requires human review for technical accuracy
- May need updates as ROS 2 evolves
- Should not replace subject matter expert validation
- Generated code examples need testing in actual environment

## Related Skills

- **RAG Setup**: Ingests generated chapters into vector database
- **Auth Setup**: Can generate authentication examples for tutorials
- **Diagram Describer**: Complements content with visual explanations
