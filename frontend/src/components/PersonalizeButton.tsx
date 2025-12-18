/**
 * PersonalizeButton Component
 *
 * Provides complexity level controls for chapter personalization.
 * Integrates with backend /api/v1/personalize-chapter endpoint.
 */

import React, { useState, useEffect } from 'react';
import { apiRequest } from '../utils/apiClient';

type ComplexityLevel = 'beginner' | 'intermediate' | 'advanced';

interface PersonalizeButtonProps {
  chapterId: string;
  onContentChange: (content: string, level: ComplexityLevel) => void;
  originalContent: string;
}

interface PersonalizeResponse {
  content: string;
  cached: boolean;
  generation_time_ms: number;
  chapter_id: string;
  complexity_level: string;
}

export default function PersonalizeButton({
  chapterId,
  onContentChange,
  originalContent
}: PersonalizeButtonProps) {
  const [currentLevel, setCurrentLevel] = useState<ComplexityLevel>('intermediate');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showSuccess, setShowSuccess] = useState(false);

  // Load saved preference from localStorage on mount
  useEffect(() => {
    const savedLevel = localStorage.getItem(`chapter_${chapterId}_complexity`) as ComplexityLevel;
    if (savedLevel && ['beginner', 'intermediate', 'advanced'].includes(savedLevel)) {
      setCurrentLevel(savedLevel);
      // Auto-apply saved preference
      if (savedLevel !== 'intermediate') {
        handlePersonalize(savedLevel);
      }
    }
  }, [chapterId]);

  const handlePersonalize = async (level: ComplexityLevel) => {
    if (level === currentLevel && level !== 'intermediate') {
      return; // Already at this level
    }

    setLoading(true);
    setError(null);
    setShowSuccess(false);

    try {
      // For intermediate, just reset to original
      if (level === 'intermediate') {
        onContentChange(originalContent, level);
        setCurrentLevel(level);
        localStorage.setItem(`chapter_${chapterId}_complexity`, level);
        setShowSuccess(true);
        setTimeout(() => setShowSuccess(false), 2000);
        return;
      }

      // Call personalization API
      const response = await apiRequest(`/api/v1/personalize-chapter`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          complexity_level: level,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to personalize chapter');
      }

      const data: PersonalizeResponse = await response.json();

      // Update content and state
      onContentChange(data.content, level);
      setCurrentLevel(level);

      // Save preference to localStorage
      localStorage.setItem(`chapter_${chapterId}_complexity`, level);

      // Show success message
      setShowSuccess(true);
      setTimeout(() => setShowSuccess(false), 2000);

    } catch (err) {
      console.error('Personalization error:', err);
      setError(err instanceof Error ? err.message : 'Failed to personalize chapter');
    } finally {
      setLoading(false);
    }
  };

  const handleReset = () => {
    handlePersonalize('intermediate');
  };

  return (
    <div className="personalize-controls">
      <div className="personalize-header">
        <span className="personalize-label">Adjust Complexity:</span>
        {showSuccess && (
          <span className="personalize-success">✓ Applied</span>
        )}
      </div>

      <div className="personalize-buttons">
        <button
          className={`personalize-btn ${currentLevel === 'beginner' ? 'active' : ''}`}
          onClick={() => handlePersonalize('beginner')}
          disabled={loading}
        >
          Beginner
        </button>

        <button
          className={`personalize-btn ${currentLevel === 'intermediate' ? 'active' : ''}`}
          onClick={() => handlePersonalize('intermediate')}
          disabled={loading}
        >
          Intermediate
        </button>

        <button
          className={`personalize-btn ${currentLevel === 'advanced' ? 'active' : ''}`}
          onClick={() => handlePersonalize('advanced')}
          disabled={loading}
        >
          Advanced
        </button>

        {currentLevel !== 'intermediate' && (
          <button
            className="personalize-reset"
            onClick={handleReset}
            disabled={loading}
            title="Reset to default complexity"
          >
            ↺ Reset
          </button>
        )}
      </div>

      {loading && (
        <div className="personalize-loading">
          <span className="spinner"></span>
          <span>Personalizing chapter...</span>
        </div>
      )}

      {error && (
        <div className="personalize-error">
          ⚠ {error}
        </div>
      )}
    </div>
  );
}
