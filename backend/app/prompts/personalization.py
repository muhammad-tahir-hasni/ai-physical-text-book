"""
Personalization prompts for adjusting chapter difficulty levels.

Three complexity levels:
- Beginner: Simple language, everyday analogies, step-by-step explanations
- Intermediate: Balanced technical content (original chapter level)
- Advanced: Precise terminology, mathematical formulations, research references
"""

BEGINNER_PROMPT = """Rewrite the following robotics chapter for absolute beginners:

GUIDELINES:
- Use simple, everyday language and analogies (e.g., "ROS 2 topics are like radio channels")
- Explain all technical terms immediately when first mentioned
- Include step-by-step walkthroughs with "Why does this matter?" context
- Add concrete examples from everyday life
- Break down complex concepts into digestible pieces
- Use encouraging, friendly tone
- Avoid jargon; when unavoidable, define it in plain English

TARGET AUDIENCE: No prior robotics knowledge, basic programming familiarity

ORIGINAL CHAPTER:
{chapter_content}

REWRITTEN CHAPTER (maintain structure and length ~900-1200 words):"""

INTERMEDIATE_PROMPT = """{chapter_content}"""

ADVANCED_PROMPT = """Rewrite the following robotics chapter for advanced practitioners:

GUIDELINES:
- Use precise technical terminology without over-explanation
- Assume strong CS/engineering background (data structures, algorithms, systems)
- Reference advanced concepts: control theory, optimization, machine learning
- Include mathematical formulations where relevant (equations, complexity analysis)
- Cite cutting-edge research papers from last 3 years
- Discuss trade-offs, edge cases, failure modes, and limitations
- Mention production considerations (scalability, performance, debugging)
- Be concise and technical

TARGET AUDIENCE: Experienced developers/researchers with strong technical foundation

ORIGINAL CHAPTER:
{chapter_content}

REWRITTEN CHAPTER (maintain structure and length ~900-1200 words):"""


def get_personalization_prompt(complexity_level: str, chapter_content: str) -> str:
    """
    Get the appropriate personalization prompt for the given complexity level.

    Args:
        complexity_level: One of "beginner", "intermediate", "advanced"
        chapter_content: Original chapter markdown content

    Returns:
        Formatted prompt ready for LLM

    Raises:
        ValueError: If complexity_level is invalid
    """
    prompts = {
        "beginner": BEGINNER_PROMPT,
        "intermediate": INTERMEDIATE_PROMPT,
        "advanced": ADVANCED_PROMPT
    }

    if complexity_level not in prompts:
        raise ValueError(
            f"Invalid complexity level: {complexity_level}. "
            f"Must be one of: {', '.join(prompts.keys())}"
        )

    return prompts[complexity_level].format(chapter_content=chapter_content)
