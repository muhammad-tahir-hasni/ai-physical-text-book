/**
 * Custom hook for handling text selection and "Ask about this" tooltip.
 *
 * Detects when user selects text and shows a tooltip to query the chatbot
 * about the selected content.
 */

import { useEffect, useState, useCallback } from 'react';
import { useChat } from '../context/ChatContext';

interface SelectionPosition {
  top: number;
  left: number;
}

export function useTextSelection() {
  const [selectedText, setSelectedText] = useState('');
  const [showTooltip, setShowTooltip] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState<SelectionPosition>({ top: 0, left: 0 });

  const { sendMessage, openChat } = useChat();

  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length > 3 && text.length < 500) {
      // Get selection position for tooltip
      const range = selection?.getRangeAt(0);
      const rect = range?.getBoundingClientRect();

      if (rect) {
        setSelectedText(text);
        setShowTooltip(true);
        setTooltipPosition({
          top: rect.top + window.scrollY - 45, // Position above selection
          left: rect.left + window.scrollX + rect.width / 2, // Center horizontally
        });
      }
    } else {
      setShowTooltip(false);
      setSelectedText('');
    }
  }, []);

  const handleClickOutside = useCallback((event: MouseEvent) => {
    // Hide tooltip when clicking outside
    const tooltip = document.getElementById('text-selection-tooltip');
    if (tooltip && !tooltip.contains(event.target as Node)) {
      setShowTooltip(false);
      setSelectedText('');
    }
  }, []);

  const askAboutSelection = useCallback(async () => {
    if (selectedText) {
      // Get surrounding context (paragraph)
      const selection = window.getSelection();
      const anchorNode = selection?.anchorNode;

      let context = '';
      if (anchorNode) {
        // Try to get parent paragraph or container
        let parent: Element | null = anchorNode.parentElement;
        while (parent && parent.tagName !== 'P' && parent.tagName !== 'DIV') {
          parent = parent.parentElement;
        }
        if (parent) {
          context = parent.textContent?.substring(0, 1000) || '';
        }
      }

      // Build query
      const query = `Explain or elaborate on: "${selectedText}"`;

      // Open chat and send message
      openChat();
      await sendMessage(query);

      // Clear selection
      setShowTooltip(false);
      setSelectedText('');
      window.getSelection()?.removeAllRanges();
    }
  }, [selectedText, sendMessage, openChat]);

  useEffect(() => {
    // Add selection listener
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    // Add click outside listener
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [handleSelection, handleClickOutside]);

  return {
    selectedText,
    showTooltip,
    tooltipPosition,
    askAboutSelection,
  };
}
