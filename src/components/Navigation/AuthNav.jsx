import React from 'react';
import { Link, useLocation } from 'react-router-dom';
import { useAuth } from '../Auth/AuthProvider';
import './AuthNav.css'; // Create this CSS file

const AuthNav = () => {
  const { user, isAuthenticated, logout } = useAuth();
  const location = useLocation();

  // Don't show auth nav on auth pages
  if (location.pathname.startsWith('/auth/') ||
      location.pathname === '/auth/signin' ||
      location.pathname === '/auth/signup') {
    return null;
  }

  return (
    <nav className="auth-nav">
      <div className="auth-nav-container">
        {isAuthenticated ? (
          <div className="auth-nav-authenticated">
            <span className="user-greeting">
              Welcome, {user?.first_name || user?.email}!
            </span>
            <Link to="/profile" className="nav-link">
              Profile
            </Link>
            <button onClick={logout} className="logout-button">
              Logout
            </button>
          </div>
        ) : (
          <div className="auth-nav-anonymous">
            <Link to="/auth/signin" className="nav-link">
              Sign In
            </Link>
            <Link to="/auth/signup" className="nav-link nav-link-signup">
              Sign Up
            </Link>
          </div>
        )}
      </div>
    </nav>
  );
};

export default AuthNav;