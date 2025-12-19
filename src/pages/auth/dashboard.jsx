import React from 'react';
import { useAuth } from '../../components/Auth/AuthProvider';
import { useHistory } from 'react-router-dom';
import '../../components/Auth/Auth.css';

const DashboardPage = () => {
  const { user, logout } = useAuth();
  const history = useHistory();

  const handleLogout = () => {
    logout();
    history.push('/auth/signin');
  };

  return (
    <div className="page-container">
      <div className="dashboard-page">
        <header className="dashboard-header">
          <h1>Welcome to Your Dashboard</h1>
          <button className="logout-button" onClick={handleLogout}>
            Logout
          </button>
        </header>

        <div className="dashboard-content">
          <div className="user-info-card">
            <h2>Hello, {user?.first_name || user?.email || 'User'}!</h2>
            <p>Your account is ready to use.</p>

            {user && (
              <div className="user-details">
                <p><strong>Email:</strong> {user.email}</p>
                <p><strong>Member since:</strong> {user.created_at ? new Date(user.created_at).toLocaleDateString() : 'N/A'}</p>
              </div>
            )}
          </div>

          <div className="dashboard-features">
            <h3>Quick Access</h3>
            <div className="feature-grid">
              <div className="feature-card">
                <h4>Documentation</h4>
                <p>Access the complete robotics and AI curriculum</p>
                <a href="/docs/" className="feature-link">Go to Docs</a>
              </div>
              <div className="feature-card">
                <h4>Profile Settings</h4>
                <p>Update your learning preferences and profile</p>
                <a href="/auth/profile" className="feature-link">Edit Profile</a>
              </div>
              <div className="feature-card">
                <h4>AI Chat</h4>
                <p>Get help with robotics and AI concepts</p>
                <a href="/" className="feature-link">Open Chat</a>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DashboardPage;