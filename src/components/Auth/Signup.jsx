import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import '../Auth/Auth.css'; // Basic auth styling

const Signup = () => {
  const { signUp, error, isLoading } = useAuth();
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
  });

  const [profileData, setProfileData] = useState({
    programming_level: '',
    programming_languages: [],
    ai_knowledge_level: '',
    hardware_experience: '',
    learning_style: '',
  });

  const [showPasswordMismatch, setShowPasswordMismatch] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value,
    });
  };

  const handleProfileChange = (e) => {
    const { name, value, type, checked } = e.target;

    if (type === 'checkbox') {
      // Handle multiple selection for programming languages
      if (name === 'programming_languages') {
        setProfileData(prev => {
          const currentLanguages = [...prev.programming_languages];
          if (checked) {
            currentLanguages.push(value);
          } else {
            const index = currentLanguages.indexOf(value);
            if (index > -1) {
              currentLanguages.splice(index, 1);
            }
          }
          return { ...prev, programming_languages: currentLanguages };
        });
      }
    } else {
      setProfileData({
        ...profileData,
        [name]: value,
      });
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Check if passwords match
    if (formData.password !== formData.confirmPassword) {
      setShowPasswordMismatch(true);
      return;
    }

    setShowPasswordMismatch(false);

    // Validate required profile fields
    if (!profileData.programming_level || !profileData.ai_knowledge_level ||
        !profileData.hardware_experience || !profileData.learning_style) {
      alert('Please fill in all required profile information.');
      return;
    }

    const result = await signUp(formData.email, formData.password, profileData);

    if (result.success) {
      // Redirect to dashboard or home page after successful signup
      window.location.href = '/';
    }
  };

  // Programming languages options
  const programmingLanguages = [
    'Python', 'C++', 'JavaScript', 'TypeScript', 'Java',
    'C#', 'Rust', 'Go', 'MATLAB', 'ROS', 'Other'
  ];

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Create Account</h2>

        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSubmit}>
          {/* Email and Password fields */}
          <div className="form-group">
            <label htmlFor="email">Email:</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="password">Password:</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="confirmPassword">Confirm Password:</label>
            <input
              type="password"
              id="confirmPassword"
              name="confirmPassword"
              value={formData.confirmPassword}
              onChange={handleChange}
              required
            />
            {showPasswordMismatch && (
              <div className="error-message">Passwords do not match</div>
            )}
          </div>

          {/* Profile Information Section */}
          <div className="profile-section">
            <h3>User Profile</h3>
            <p>Please provide information about your background to personalize your experience:</p>

            {/* Programming Level */}
            <div className="form-group">
              <label id="programming-level-label">Programming Level:</label>
              <div className="radio-group" role="radiogroup" aria-labelledby="programming-level-label">
                {['Beginner', 'Intermediate', 'Advanced'].map(level => (
                  <label key={level} className="radio-option" htmlFor={`programming_level_${level.toLowerCase()}`}>
                    <input
                      type="radio"
                      id={`programming_level_${level.toLowerCase()}`}
                      name="programming_level"
                      value={level}
                      checked={profileData.programming_level === level}
                      onChange={handleProfileChange}
                      required
                    />
                    {level}
                  </label>
                ))}
              </div>
            </div>

            {/* Programming Languages */}
            <div className="form-group">
              <label id="programming-languages-label">Known Programming Languages (select all that apply):</label>
              <div className="checkbox-group" role="group" aria-labelledby="programming-languages-label">
                {programmingLanguages.map(lang => (
                  <label key={lang} className="checkbox-option" htmlFor={`programming_language_${lang.toLowerCase().replace(/\s+/g, '_')}`}>
                    <input
                      type="checkbox"
                      id={`programming_language_${lang.toLowerCase().replace(/\s+/g, '_')}`}
                      name="programming_languages"
                      value={lang}
                      checked={profileData.programming_languages.includes(lang)}
                      onChange={handleProfileChange}
                    />
                    {lang}
                  </label>
                ))}
              </div>
            </div>

            {/* AI Knowledge Level */}
            <div className="form-group">
              <label id="ai-knowledge-level-label">AI Knowledge Level:</label>
              <div className="radio-group" role="radiogroup" aria-labelledby="ai-knowledge-level-label">
                {['None', 'Basic', 'Intermediate', 'Advanced'].map(level => (
                  <label key={level} className="radio-option" htmlFor={`ai_knowledge_level_${level.toLowerCase()}`}>
                    <input
                      type="radio"
                      id={`ai_knowledge_level_${level.toLowerCase()}`}
                      name="ai_knowledge_level"
                      value={level}
                      checked={profileData.ai_knowledge_level === level}
                      onChange={handleProfileChange}
                      required
                    />
                    {level}
                  </label>
                ))}
              </div>
            </div>

            {/* Hardware Experience */}
            <div className="form-group">
              <label id="hardware-experience-label">Hardware Experience:</label>
              <div className="radio-group" role="radiogroup" aria-labelledby="hardware-experience-label">
                {['None', 'Basic', 'Robotics', 'Embedded Systems'].map(exp => (
                  <label key={exp} className="radio-option" htmlFor={`hardware_experience_${exp.toLowerCase().replace(/\s+/g, '_')}`}>
                    <input
                      type="radio"
                      id={`hardware_experience_${exp.toLowerCase().replace(/\s+/g, '_')}`}
                      name="hardware_experience"
                      value={exp}
                      checked={profileData.hardware_experience === exp}
                      onChange={handleProfileChange}
                      required
                    />
                    {exp}
                  </label>
                ))}
              </div>
            </div>

            {/* Learning Style */}
            <div className="form-group">
              <label id="learning-style-label">Preferred Learning Style:</label>
              <div className="radio-group" role="radiogroup" aria-labelledby="learning-style-label">
                {['Theory', 'Code-first', 'Visual', 'Mixed'].map(style => (
                  <label key={style} className="radio-option" htmlFor={`learning_style_${style.toLowerCase().replace('-', '_')}`}>
                    <input
                      type="radio"
                      id={`learning_style_${style.toLowerCase().replace('-', '_')}`}
                      name="learning_style"
                      value={style}
                      checked={profileData.learning_style === style}
                      onChange={handleProfileChange}
                      required
                    />
                    {style}
                  </label>
                ))}
              </div>
            </div>
          </div>

          <button type="submit" disabled={isLoading} className="submit-btn">
            {isLoading ? 'Creating Account...' : 'Sign Up'}
          </button>
        </form>

        <div className="auth-links">
          <p>Already have an account? <a href="/auth/signin">Sign in</a></p>
        </div>
      </div>
    </div>
  );
};

export default Signup;