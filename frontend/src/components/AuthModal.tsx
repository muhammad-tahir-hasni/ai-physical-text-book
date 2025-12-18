/**
 * AuthModal component with login/signup tabs and form validation.
 *
 * Features:
 * - Tab switcher (Login / Signup)
 * - Form validation (email format, password strength)
 * - Password strength indicator
 * - Error message display
 * - Background questionnaire after signup
 */

import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import BackgroundQuestionnaire from './BackgroundQuestionnaire';
import '../css/auth.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  initialMode?: 'login' | 'signup';
}

export default function AuthModal({ isOpen, onClose, initialMode = 'login' }: AuthModalProps) {
  const { login, signup } = useAuth();
  const [mode, setMode] = useState<'login' | 'signup'>(initialMode);
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  if (!isOpen) return null;

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const getPasswordStrength = (password: string): { score: number; label: string; color: string } => {
    let score = 0;
    if (password.length >= 8) score++;
    if (password.length >= 12) score++;
    if (/[a-z]/.test(password)) score++;
    if (/[A-Z]/.test(password)) score++;
    if (/[0-9]/.test(password)) score++;
    if (/[^a-zA-Z0-9]/.test(password)) score++;

    if (score <= 2) return { score, label: 'Weak', color: '#ef4444' };
    if (score <= 4) return { score, label: 'Medium', color: '#f59e0b' };
    return { score, label: 'Strong', color: '#10b981' };
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    // Validation
    if (!validateEmail(email)) {
      setError('Please enter a valid email address');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    if (mode === 'signup') {
      if (!/[A-Z]/.test(password)) {
        setError('Password must contain at least one uppercase letter');
        return;
      }
      if (!/[0-9]/.test(password)) {
        setError('Password must contain at least one number');
        return;
      }
      if (password !== confirmPassword) {
        setError('Passwords do not match');
        return;
      }
    }

    setIsLoading(true);

    try {
      if (mode === 'login') {
        await login(email, password);
        onClose();
      } else {
        // Show questionnaire after signup
        setShowQuestionnaire(true);
      }
    } catch (err: any) {
      setError(err.message || 'Authentication failed');
    } finally {
      setIsLoading(false);
    }
  };

  const handleQuestionnaireSubmit = async (profile: any) => {
    setIsLoading(true);
    try {
      await signup(email, password, profile);
      setShowQuestionnaire(false);
      onClose();
    } catch (err: any) {
      setError(err.message || 'Signup failed');
    } finally {
      setIsLoading(false);
    }
  };

  const handleQuestionnaireSkip = async () => {
    setIsLoading(true);
    try {
      await signup(email, password, null); // No profile data
      setShowQuestionnaire(false);
      onClose();
    } catch (err: any) {
      setError(err.message || 'Signup failed');
    } finally {
      setIsLoading(false);
    }
  };

  const passwordStrength = mode === 'signup' ? getPasswordStrength(password) : null;

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div className="auth-modal-content" onClick={(e) => e.stopPropagation()}>
        {showQuestionnaire ? (
          <BackgroundQuestionnaire
            onSubmit={handleQuestionnaireSubmit}
            onSkip={handleQuestionnaireSkip}
            isLoading={isLoading}
          />
        ) : (
          <>
            <button className="auth-modal-close" onClick={onClose}>
              ×
            </button>

            <div className="auth-modal-tabs">
              <button
                className={`auth-tab ${mode === 'login' ? 'active' : ''}`}
                onClick={() => setMode('login')}
              >
                Login
              </button>
              <button
                className={`auth-tab ${mode === 'signup' ? 'active' : ''}`}
                onClick={() => setMode('signup')}
              >
                Sign Up
              </button>
            </div>

            <form onSubmit={handleSubmit} className="auth-form">
              <div className="auth-form-group">
                <label htmlFor="email">Email</label>
                <input
                  id="email"
                  type="email"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  placeholder="you@example.com"
                  required
                />
              </div>

              <div className="auth-form-group">
                <label htmlFor="password">Password</label>
                <input
                  id="password"
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="••••••••"
                  required
                  minLength={8}
                />
                {mode === 'signup' && password && passwordStrength && (
                  <div className="password-strength">
                    <div
                      className="password-strength-bar"
                      style={{
                        width: `${(passwordStrength.score / 6) * 100}%`,
                        backgroundColor: passwordStrength.color,
                      }}
                    />
                    <span style={{ color: passwordStrength.color }}>
                      {passwordStrength.label}
                    </span>
                  </div>
                )}
              </div>

              {mode === 'signup' && (
                <div className="auth-form-group">
                  <label htmlFor="confirmPassword">Confirm Password</label>
                  <input
                    id="confirmPassword"
                    type="password"
                    value={confirmPassword}
                    onChange={(e) => setConfirmPassword(e.target.value)}
                    placeholder="••••••••"
                    required
                    minLength={8}
                  />
                </div>
              )}

              {error && <div className="auth-error">{error}</div>}

              <button type="submit" className="auth-submit" disabled={isLoading}>
                {isLoading ? 'Loading...' : mode === 'login' ? 'Log In' : 'Next'}
              </button>

              {mode === 'signup' && (
                <div>
                  <p className="auth-requirements">
                    Password must be at least 8 characters with 1 uppercase letter and 1 number
                  </p>
                  <p className="auth-requirements" style={{ marginTop: '8px', fontSize: '13px', color: '#666' }}>
                    Click "Next" to continue with optional background questionnaire
                  </p>
                </div>
              )}
            </form>
          </>
        )}
      </div>
    </div>
  );
}
