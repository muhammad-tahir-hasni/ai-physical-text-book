"""
Text chunking utility for RAG system.

Splits documents into overlapping chunks optimized for embedding and retrieval.
"""

import re
from typing import List


def chunk_text(
    text: str,
    chunk_size: int = 512,
    overlap: int = 50,
    min_chunk_size: int = 100
) -> List[str]:
    """
    Split text into overlapping chunks for embedding.

    Args:
        text: Input text to chunk
        chunk_size: Target chunk size in tokens (approximate using word count)
        overlap: Number of tokens to overlap between chunks
        min_chunk_size: Minimum chunk size to avoid tiny fragments

    Returns:
        List of text chunks

    Example:
        >>> text = "ROS 2 is a middleware framework..." * 100
        >>> chunks = chunk_text(text, chunk_size=512, overlap=50)
        >>> len(chunks)
        5
    """
    if not text or not text.strip():
        return []

    # Normalize whitespace
    text = re.sub(r'\s+', ' ', text.strip())

    # Approximate tokens as words (rough heuristic: 1 token ≈ 0.75 words)
    # For more accuracy, we'd use tiktoken, but this avoids extra dependencies
    words = text.split()
    token_multiplier = 0.75  # tokens per word approximation

    chunk_size_words = int(chunk_size / token_multiplier)
    overlap_words = int(overlap / token_multiplier)
    min_chunk_words = int(min_chunk_size / token_multiplier)

    chunks = []
    start = 0

    while start < len(words):
        # Get chunk of words
        end = min(start + chunk_size_words, len(words))
        chunk_words = words[start:end]

        # Skip if chunk is too small (except for last chunk)
        if len(chunk_words) < min_chunk_words and end < len(words):
            start += chunk_size_words - overlap_words
            continue

        # Join words back into text
        chunk = ' '.join(chunk_words)
        chunks.append(chunk)

        # Move start position with overlap
        if end >= len(words):
            break
        start += chunk_size_words - overlap_words

    return chunks


def chunk_markdown(
    markdown_text: str,
    chunk_size: int = 512,
    overlap: int = 50,
    preserve_structure: bool = True
) -> List[dict]:
    """
    Chunk markdown content while preserving document structure.

    Attempts to split on section boundaries (##, ###) when possible
    to maintain semantic coherence.

    Args:
        markdown_text: Markdown content to chunk
        chunk_size: Target chunk size in tokens
        overlap: Overlap between chunks
        preserve_structure: Try to split on section boundaries

    Returns:
        List of dicts with 'text' and 'metadata' (heading path)

    Example:
        >>> md = "# Chapter 1\\n## Section 1.1\\nContent here..."
        >>> chunks = chunk_markdown(md)
        >>> chunks[0]['metadata']['heading']
        'Chapter 1 > Section 1.1'
    """
    if not markdown_text or not markdown_text.strip():
        return []

    chunks_with_metadata = []

    if preserve_structure:
        # Split by sections (headings)
        sections = re.split(r'(^#{1,6}\s+.+$)', markdown_text, flags=re.MULTILINE)

        current_heading_stack = []
        current_content = []

        for i, section in enumerate(sections):
            if not section.strip():
                continue

            # Check if this is a heading
            heading_match = re.match(r'^(#{1,6})\s+(.+)$', section.strip())

            if heading_match:
                # Process accumulated content before new heading
                if current_content:
                    content_text = '\n'.join(current_content)
                    text_chunks = chunk_text(content_text, chunk_size, overlap)

                    for chunk in text_chunks:
                        chunks_with_metadata.append({
                            'text': chunk,
                            'metadata': {
                                'heading': ' > '.join(current_heading_stack) if current_heading_stack else 'Introduction',
                                'level': len(current_heading_stack)
                            }
                        })
                    current_content = []

                # Update heading stack
                level = len(heading_match.group(1))
                heading_text = heading_match.group(2).strip()

                # Pop headings of same or deeper level
                current_heading_stack = [h for h, l in
                    [(h, l) for h, l in zip(current_heading_stack, range(len(current_heading_stack)))]
                    if l < level - 1]

                # Add new heading
                if len(current_heading_stack) < level:
                    current_heading_stack.append(heading_text)
                else:
                    current_heading_stack = current_heading_stack[:level-1] + [heading_text]
            else:
                # Accumulate content
                current_content.append(section)

        # Process any remaining content
        if current_content:
            content_text = '\n'.join(current_content)
            text_chunks = chunk_text(content_text, chunk_size, overlap)

            for chunk in text_chunks:
                chunks_with_metadata.append({
                    'text': chunk,
                    'metadata': {
                        'heading': ' > '.join(current_heading_stack) if current_heading_stack else 'Introduction',
                        'level': len(current_heading_stack)
                    }
                })
    else:
        # Simple chunking without structure preservation
        text_chunks = chunk_text(markdown_text, chunk_size, overlap)
        chunks_with_metadata = [
            {'text': chunk, 'metadata': {'heading': 'Document', 'level': 0}}
            for chunk in text_chunks
        ]

    return chunks_with_metadata


def extract_code_blocks(markdown_text: str) -> List[dict]:
    """
    Extract code blocks from markdown for separate indexing.

    Args:
        markdown_text: Markdown content

    Returns:
        List of dicts with 'code', 'language', and 'context'
    """
    code_pattern = r'```(\w+)?\n(.*?)```'
    matches = re.findall(code_pattern, markdown_text, re.DOTALL)

    code_blocks = []
    for language, code in matches:
        code_blocks.append({
            'code': code.strip(),
            'language': language or 'text',
            'context': _get_surrounding_text(markdown_text, code)
        })

    return code_blocks


def _get_surrounding_text(full_text: str, code_snippet: str, context_words: int = 50) -> str:
    """Get text surrounding a code snippet for context."""
    idx = full_text.find(code_snippet)
    if idx == -1:
        return ""

    # Get text before code
    before_text = full_text[:idx]
    before_words = before_text.split()[-context_words:]

    # Get text after code
    after_text = full_text[idx + len(code_snippet):]
    after_words = after_text.split()[:context_words]

    return ' '.join(before_words) + ' [CODE] ' + ' '.join(after_words)
