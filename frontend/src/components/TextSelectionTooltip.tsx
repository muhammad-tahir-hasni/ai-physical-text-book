/**
 * TextSelectionTooltip component.
 *
 * Shows "Ask about this" tooltip when text is selected on the page.
 */

import React from 'react';
import { useTextSelection } from '../hooks/useTextSelection';
import styles from './TextSelectionTooltip.module.css';

export function TextSelectionTooltip() {
  const { showTooltip, tooltipPosition, askAboutSelection } = useTextSelection();

  if (!showTooltip) {
    return null;
  }

  return (
    <div
      id="text-selection-tooltip"
      className={styles.tooltip}
      style={{
        top: `${tooltipPosition.top}px`,
        left: `${tooltipPosition.left}px`,
      }}
    >
      <button
        className={styles.tooltipButton}
        onClick={askAboutSelection}
      >
        <span className={styles.icon}>💬</span>
        <span className={styles.text}>Ask about this</span>
      </button>
    </div>
  );
}
