import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import '../Auth/Auth.css'; // Assuming we have a shared CSS file

const SignupForm = () => {
  const { signup, error, clearError } = useAuth();
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    first_name: '',
    last_name: '',
    programming_level: '',
    programming_languages: [],
    ai_knowledge_level: '',
    hardware_experience: '',
    learning_style: '',
    agree_to_terms: false
  });
  const [loading, setLoading] = useState(false);

  // Programming languages options
  const programmingLanguagesOptions = [
    "Python", "JavaScript", "TypeScript", "C++", "Java",
    "C#", "Rust", "Go", "Other"
  ];

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

    // Clear error when user starts typing
    if (error) clearError();
  };

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);

    try {
      const result = await signup(formData);
      if (result.success) {
        // Use window.location for navigation (works with Docusaurus)
        if (result.requiresEmailVerification) {
          window.location.href = '/auth/verify-email';
        } else {
          window.location.href = '/docs/';
        }
      } else {
        console.error('Signup failed:', result.error);
      }
    } catch (err) {
      console.error('Signup error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Create Account</h2>

        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSubmit}>
          {/* Basic Information */}
          <div className="form-group">
            <label htmlFor="email">Email *</label>
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
            <label htmlFor="password">Password *</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              required
              minLength="8"
              placeholder="At least 8 characters with uppercase, lowercase, number, and symbol"
            />
          </div>

          <div className="form-row">
            <div className="form-group">
              <label htmlFor="first_name">First Name *</label>
              <input
                type="text"
                id="first_name"
                name="first_name"
                value={formData.first_name}
                onChange={handleChange}
                required
              />
            </div>

            <div className="form-group">
              <label htmlFor="last_name">Last Name *</label>
              <input
                type="text"
                id="last_name"
                name="last_name"
                value={formData.last_name}
                onChange={handleChange}
                required
              />
            </div>
          </div>

          {/* Programming Level */}
          <div className="form-group">
            <label htmlFor="programming_level">Programming Level *</label>
            <select
              id="programming_level"
              name="programming_level"
              value={formData.programming_level}
              onChange={handleChange}
              required
            >
              <option value="">Select your level</option>
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          {/* Programming Languages */}
          <div className="form-group">
            <label>Programming Languages You Know * (Select all that apply)</label>
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

          {/* AI Knowledge Level */}
          <div className="form-group">
            <label htmlFor="ai_knowledge_level">AI Knowledge Level *</label>
            <select
              id="ai_knowledge_level"
              name="ai_knowledge_level"
              value={formData.ai_knowledge_level}
              onChange={handleChange}
              required
            >
              <option value="">Select your level</option>
              <option value="none">None</option>
              <option value="basic">Basic</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          {/* Hardware Experience */}
          <div className="form-group">
            <label htmlFor="hardware_experience">Hardware Experience *</label>
            <select
              id="hardware_experience"
              name="hardware_experience"
              value={formData.hardware_experience}
              onChange={handleChange}
              required
            >
              <option value="">Select your experience</option>
              <option value="none">None</option>
              <option value="basic">Basic</option>
              <option value="robotics">Robotics</option>
              <option value="embedded_systems">Embedded Systems</option>
            </select>
          </div>

          {/* Learning Style */}
          <div className="form-group">
            <label htmlFor="learning_style">Preferred Learning Style *</label>
            <select
              id="learning_style"
              name="learning_style"
              value={formData.learning_style}
              onChange={handleChange}
              required
            >
              <option value="">Select your style</option>
              <option value="theory">Theory First</option>
              <option value="code_first">Code First</option>
              <option value="visual">Visual Learner</option>
              <option value="mixed">Mixed Approach</option>
            </select>
          </div>

          {/* Terms Agreement */}
          <div className="form-group">
            <label className="checkbox-label">
              <input
                type="checkbox"
                name="agree_to_terms"
                checked={formData.agree_to_terms}
                onChange={handleChange}
                required
              />
              I agree to the Terms of Service and Privacy Policy *
            </label>
          </div>

          <button
            type="submit"
            className="auth-button"
            disabled={loading || !formData.agree_to_terms}
          >
            {loading ? 'Creating Account...' : 'Sign Up'}
          </button>
        </form>

        <div className="auth-links">
          <p>
            Already have an account? <a href="/auth/signin">Sign in</a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default SignupForm;