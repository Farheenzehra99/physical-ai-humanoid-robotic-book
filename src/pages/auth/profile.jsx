import React, { useState, useEffect } from 'react';
import { useAuth } from '../../components/Auth/AuthProvider';
import '../../components/Auth/Auth.css'; // Page-specific styles

const ProfilePage = () => {
  const { user, getUserProfile, updateUserProfile, loading } = useAuth();
  const [profile, setProfile] = useState(null);
  const [editMode, setEditMode] = useState(false);
  const [formData, setFormData] = useState({
    programming_level: '',
    programming_languages: [],
    ai_knowledge_level: '',
    hardware_experience: '',
    learning_style: ''
  });
  const [updateStatus, setUpdateStatus] = useState({ success: false, error: null });

  // Programming languages options
  const programmingLanguagesOptions = [
    "Python", "JavaScript", "TypeScript", "C++", "Java",
    "C#", "Rust", "Go", "Other"
  ];

  // Load profile data when component mounts
  useEffect(() => {
    const loadProfile = async () => {
      const profileData = await getUserProfile();
      if (profileData) {
        setProfile(profileData);
        setFormData({
          programming_level: profileData.programming_level || '',
          programming_languages: profileData.programming_languages || [],
          ai_knowledge_level: profileData.ai_knowledge_level || '',
          hardware_experience: profileData.hardware_experience || '',
          learning_style: profileData.learning_style || ''
        });
      }
    };

    if (user) {
      loadProfile();
    }
  }, [user]);

  // Handle input changes
  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;

    if (name === 'programming_languages') {
      // Handle multiple selection for programming languages
      const newLanguages = [...formData.programming_languages];
      if (checked && !newLanguages.includes(value)) {
        newLanguages.push(value);
      } else if (!checked) {
        const index = newLanguages.indexOf(value);
        if (index > -1) {
          newLanguages.splice(index, 1);
        }
      }
      setFormData({
        ...formData,
        [name]: newLanguages
      });
    } else {
      setFormData({
        ...formData,
        [name]: type === 'checkbox' ? checked : value
      });
    }

    // Clear status when user starts editing
    if (updateStatus.success || updateStatus.error) {
      setUpdateStatus({ success: false, error: null });
    }
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      const success = await updateUserProfile(formData);
      if (success) {
        setUpdateStatus({ success: true, error: null });
        setEditMode(false);
        // Update local profile state
        setProfile(prev => ({
          ...prev,
          ...formData
        }));
      } else {
        setUpdateStatus({ success: false, error: 'Failed to update profile' });
      }
    } catch (err) {
      setUpdateStatus({ success: false, error: 'Error updating profile' });
    }
  };

  // Handle edit button click
  const handleEditClick = () => {
    setEditMode(true);
    setUpdateStatus({ success: false, error: null });
  };

  // Handle cancel edit
  const handleCancelEdit = () => {
    setEditMode(false);
    // Reset form to current profile values
    if (profile) {
      setFormData({
        programming_level: profile.programming_level || '',
        programming_languages: profile.programming_languages || [],
        ai_knowledge_level: profile.ai_knowledge_level || '',
        hardware_experience: profile.hardware_experience || '',
        learning_style: profile.learning_style || ''
      });
    }
    setUpdateStatus({ success: false, error: null });
  };

  if (loading) {
    return (
      <div className="page-container">
        <div className="loading-container">
          <div className="loading-spinner">Loading profile...</div>
        </div>
      </div>
    );
  }

  return (
    <div className="page-container">
      <div className="profile-page">
        <h1>User Profile</h1>

        {user && (
          <div className="user-info">
            <h2>{user.first_name} {user.last_name}</h2>
            <p>Email: {user.email}</p>
          </div>
        )}

        <div className="profile-content">
          <h3>Learning Profile</h3>

          {updateStatus.success && (
            <div className="success-message">
              Profile updated successfully!
            </div>
          )}

          {updateStatus.error && (
            <div className="error-message">
              {updateStatus.error}
            </div>
          )}

          {editMode ? (
            <form onSubmit={handleSubmit} className="profile-form">
              <div className="form-group">
                <label htmlFor="programming_level">Programming Level</label>
                <select
                  id="programming_level"
                  name="programming_level"
                  value={formData.programming_level}
                  onChange={handleChange}
                >
                  <option value="">Select your level</option>
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className="form-group">
                <label>Programming Languages You Know</label>
                <div className="checkbox-group">
                  {programmingLanguagesOptions.map((lang) => (
                    <label key={lang} className="checkbox-label">
                      <input
                        type="checkbox"
                        name="programming_languages"
                        value={lang}
                        checked={formData.programming_languages.includes(lang)}
                        onChange={handleChange}
                      />
                      {lang}
                    </label>
                  ))}
                </div>
              </div>

              <div className="form-group">
                <label htmlFor="ai_knowledge_level">AI Knowledge Level</label>
                <select
                  id="ai_knowledge_level"
                  name="ai_knowledge_level"
                  value={formData.ai_knowledge_level}
                  onChange={handleChange}
                >
                  <option value="">Select your level</option>
                  <option value="none">None</option>
                  <option value="basic">Basic</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="hardware_experience">Hardware Experience</label>
                <select
                  id="hardware_experience"
                  name="hardware_experience"
                  value={formData.hardware_experience}
                  onChange={handleChange}
                >
                  <option value="">Select your experience</option>
                  <option value="none">None</option>
                  <option value="basic">Basic</option>
                  <option value="robotics">Robotics</option>
                  <option value="embedded_systems">Embedded Systems</option>
                </select>
              </div>

              <div className="form-group">
                <label htmlFor="learning_style">Preferred Learning Style</label>
                <select
                  id="learning_style"
                  name="learning_style"
                  value={formData.learning_style}
                  onChange={handleChange}
                >
                  <option value="">Select your style</option>
                  <option value="theory">Theory First</option>
                  <option value="code_first">Code First</option>
                  <option value="visual">Visual Learner</option>
                  <option value="mixed">Mixed Approach</option>
                </select>
              </div>

              <div className="form-actions">
                <button type="submit" className="auth-button">
                  Save Changes
                </button>
                <button type="button" onClick={handleCancelEdit} className="cancel-button">
                  Cancel
                </button>
              </div>
            </form>
          ) : (
            <div className="profile-display">
              <div className="profile-field">
                <strong>Programming Level:</strong> {profile?.programming_level || 'Not specified'}
              </div>
              <div className="profile-field">
                <strong>Programming Languages:</strong> {profile?.programming_languages?.length > 0
                  ? profile.programming_languages.join(', ')
                  : 'None specified'}
              </div>
              <div className="profile-field">
                <strong>AI Knowledge Level:</strong> {profile?.ai_knowledge_level || 'Not specified'}
              </div>
              <div className="profile-field">
                <strong>Hardware Experience:</strong> {profile?.hardware_experience || 'Not specified'}
              </div>
              <div className="profile-field">
                <strong>Learning Style:</strong> {profile?.learning_style || 'Not specified'}
              </div>

              <button onClick={handleEditClick} className="edit-button">
                Edit Profile
              </button>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default ProfilePage;