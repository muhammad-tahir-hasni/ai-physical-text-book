/**
 * BackgroundQuestionnaire component for user profile creation.
 *
 * Collects 5 questions about user's technical background:
 * 1. Software development experience
 * 2. Python familiarity
 * 3. Robotics experience
 * 4. Hardware/electronics background
 * 5. Learning goals (free text)
 */

import React, { useState } from 'react';

interface BackgroundQuestionnaireProps {
  onSubmit: (profile: any) => void;
  onSkip: () => void;
  isLoading?: boolean;
}

export default function BackgroundQuestionnaire({
  onSubmit,
  onSkip,
  isLoading = false,
}: BackgroundQuestionnaireProps) {
  const [formData, setFormData] = useState({
    software_experience: 'intermediate',
    python_familiarity: 'basic',
    robotics_experience: 'none',
    hardware_background: 'none',
    learning_goals: '',
  });

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit(formData);
  };

  const handleChange = (field: string, value: string) => {
    setFormData((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  return (
    <div className="questionnaire-container">
      <h2>Tell us about your background</h2>
      <p className="questionnaire-subtitle">
        Help us personalize your learning experience (you can skip this step)
      </p>

      <form onSubmit={handleSubmit} className="questionnaire-form">
        {/* Question 1: Software Development Experience */}
        <div className="question-group">
          <label htmlFor="software_experience">1. Software development experience?</label>
          <select
            id="software_experience"
            value={formData.software_experience}
            onChange={(e) => handleChange('software_experience', e.target.value)}
          >
            <option value="beginner">Beginner (0-1 years)</option>
            <option value="intermediate">Intermediate (1-3 years)</option>
            <option value="advanced">Advanced (3+ years)</option>
          </select>
        </div>

        {/* Question 2: Python Familiarity */}
        <div className="question-group">
          <label htmlFor="python_familiarity">2. Python familiarity?</label>
          <select
            id="python_familiarity"
            value={formData.python_familiarity}
            onChange={(e) => handleChange('python_familiarity', e.target.value)}
          >
            <option value="none">None</option>
            <option value="basic">Basic (variables, loops, functions)</option>
            <option value="proficient">Proficient (OOP, async, libraries)</option>
            <option value="expert">Expert (frameworks, optimization, production)</option>
          </select>
        </div>

        {/* Question 3: Robotics Experience */}
        <div className="question-group">
          <label htmlFor="robotics_experience">3. Robotics experience?</label>
          <select
            id="robotics_experience"
            value={formData.robotics_experience}
            onChange={(e) => handleChange('robotics_experience', e.target.value)}
          >
            <option value="none">None</option>
            <option value="hobbyist">Hobbyist (Arduino, hobby projects)</option>
            <option value="academic">Academic (courses, research)</option>
            <option value="professional">Professional (industry work)</option>
          </select>
        </div>

        {/* Question 4: Hardware/Electronics Background */}
        <div className="question-group">
          <label htmlFor="hardware_background">4. Hardware/electronics background?</label>
          <select
            id="hardware_background"
            value={formData.hardware_background}
            onChange={(e) => handleChange('hardware_background', e.target.value)}
          >
            <option value="none">None</option>
            <option value="basic">Basic (breadboards, basic circuits)</option>
            <option value="intermediate">Intermediate (sensors, actuators, PCB design)</option>
            <option value="advanced">Advanced (embedded systems, signal processing)</option>
          </select>
        </div>

        {/* Question 5: Learning Goals */}
        <div className="question-group">
          <label htmlFor="learning_goals">5. Learning goals (optional, max 200 chars)</label>
          <textarea
            id="learning_goals"
            value={formData.learning_goals}
            onChange={(e) => handleChange('learning_goals', e.target.value)}
            placeholder="What do you want to achieve with this course?"
            maxLength={200}
            rows={3}
          />
          <span className="char-count">{formData.learning_goals.length}/200</span>
        </div>

        <div className="questionnaire-actions">
          <button
            type="button"
            onClick={onSkip}
            className="questionnaire-skip"
            disabled={isLoading}
          >
            Skip
          </button>
          <button type="submit" className="questionnaire-submit" disabled={isLoading}>
            {isLoading ? 'Saving...' : 'Complete Signup'}
          </button>
        </div>
      </form>
    </div>
  );
}
