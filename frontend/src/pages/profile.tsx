/**
 * User Profile Page
 *
 * Displays user information and background profile.
 * Allows editing profile data.
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../context/AuthContext';
import ProtectedRoute from '../components/ProtectedRoute';
import { get, patch } from '../utils/apiClient';
import '../css/auth.css';

interface Profile {
  software_experience?: string;
  python_familiarity?: string;
  robotics_experience?: string;
  hardware_background?: string;
  learning_goals?: string;
  preferred_complexity?: string;
}

export default function ProfilePage() {
  const { user } = useAuth();
  const [profile, setProfile] = useState<Profile | null>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [editedProfile, setEditedProfile] = useState<Profile>({});

  // Fetch profile on mount
  useEffect(() => {
    fetchProfile();
  }, []);

  const fetchProfile = async () => {
    try {
      setIsLoading(true);
      const response = await get('/api/v1/user/profile');

      if (!response.ok) {
        throw new Error('Failed to fetch profile');
      }

      const data = await response.json();
      setProfile(data.profile);
      setEditedProfile(data.profile || {});
    } catch (err: any) {
      setError(err.message || 'Failed to load profile');
    } finally {
      setIsLoading(false);
    }
  };

  const handleSave = async () => {
    try {
      setIsSaving(true);
      setError(null);

      const response = await patch('/api/v1/user/profile', editedProfile);

      if (!response.ok) {
        throw new Error('Failed to update profile');
      }

      const data = await response.json();
      setProfile(data.profile);
      setIsEditing(false);
    } catch (err: any) {
      setError(err.message || 'Failed to save profile');
    } finally {
      setIsSaving(false);
    }
  };

  const handleCancel = () => {
    setEditedProfile(profile || {});
    setIsEditing(false);
    setError(null);
  };

  const handleChange = (field: string, value: string) => {
    setEditedProfile((prev) => ({
      ...prev,
      [field]: value,
    }));
  };

  return (
    <Layout title="Profile" description="User profile page">
      <ProtectedRoute>
        <div className="container" style={{ maxWidth: '800px', margin: '2rem auto', padding: '0 1rem' }}>
          <h1>My Profile</h1>

          {isLoading ? (
            <div>Loading profile...</div>
          ) : (
            <>
              <div className="profile-card">
                <h2>Account Information</h2>
                <div className="profile-field">
                  <strong>Email:</strong> {user?.email}
                </div>
                <div className="profile-field">
                  <strong>Member since:</strong>{' '}
                  {user?.created_at
                    ? new Date(user.created_at).toLocaleDateString()
                    : 'Unknown'}
                </div>
              </div>

              <div className="profile-card">
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                  <h2>Background Information</h2>
                  {!isEditing && (
                    <button onClick={() => setIsEditing(true)} className="profile-edit-button">
                      Edit
                    </button>
                  )}
                </div>

                {error && <div className="auth-error">{error}</div>}

                {isEditing ? (
                  <div className="profile-edit-form">
                    <div className="question-group">
                      <label>Software Experience</label>
                      <select
                        value={editedProfile.software_experience || 'intermediate'}
                        onChange={(e) => handleChange('software_experience', e.target.value)}
                      >
                        <option value="beginner">Beginner</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="advanced">Advanced</option>
                      </select>
                    </div>

                    <div className="question-group">
                      <label>Python Familiarity</label>
                      <select
                        value={editedProfile.python_familiarity || 'basic'}
                        onChange={(e) => handleChange('python_familiarity', e.target.value)}
                      >
                        <option value="none">None</option>
                        <option value="basic">Basic</option>
                        <option value="proficient">Proficient</option>
                        <option value="expert">Expert</option>
                      </select>
                    </div>

                    <div className="question-group">
                      <label>Robotics Experience</label>
                      <select
                        value={editedProfile.robotics_experience || 'none'}
                        onChange={(e) => handleChange('robotics_experience', e.target.value)}
                      >
                        <option value="none">None</option>
                        <option value="hobbyist">Hobbyist</option>
                        <option value="academic">Academic</option>
                        <option value="professional">Professional</option>
                      </select>
                    </div>

                    <div className="question-group">
                      <label>Hardware Background</label>
                      <select
                        value={editedProfile.hardware_background || 'none'}
                        onChange={(e) => handleChange('hardware_background', e.target.value)}
                      >
                        <option value="none">None</option>
                        <option value="basic">Basic</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="advanced">Advanced</option>
                      </select>
                    </div>

                    <div className="question-group">
                      <label>Learning Goals</label>
                      <textarea
                        value={editedProfile.learning_goals || ''}
                        onChange={(e) => handleChange('learning_goals', e.target.value)}
                        maxLength={200}
                        rows={3}
                        placeholder="What do you want to achieve?"
                      />
                      <span className="char-count">
                        {(editedProfile.learning_goals || '').length}/200
                      </span>
                    </div>

                    <div className="questionnaire-actions">
                      <button onClick={handleCancel} className="questionnaire-skip" disabled={isSaving}>
                        Cancel
                      </button>
                      <button onClick={handleSave} className="questionnaire-submit" disabled={isSaving}>
                        {isSaving ? 'Saving...' : 'Save Changes'}
                      </button>
                    </div>
                  </div>
                ) : profile ? (
                  <div className="profile-display">
                    <div className="profile-field">
                      <strong>Software Experience:</strong> {profile.software_experience || 'Not set'}
                    </div>
                    <div className="profile-field">
                      <strong>Python Familiarity:</strong> {profile.python_familiarity || 'Not set'}
                    </div>
                    <div className="profile-field">
                      <strong>Robotics Experience:</strong> {profile.robotics_experience || 'Not set'}
                    </div>
                    <div className="profile-field">
                      <strong>Hardware Background:</strong> {profile.hardware_background || 'Not set'}
                    </div>
                    <div className="profile-field">
                      <strong>Learning Goals:</strong>{' '}
                      {profile.learning_goals || 'Not set'}
                    </div>
                    <div className="profile-field">
                      <strong>Preferred Complexity:</strong>{' '}
                      {profile.preferred_complexity || 'intermediate'}
                    </div>
                  </div>
                ) : (
                  <p>No profile information available. Click Edit to add your background.</p>
                )}
              </div>
            </>
          )}
        </div>
      </ProtectedRoute>
    </Layout>
  );
}
