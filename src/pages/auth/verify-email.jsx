import React from 'react';
import { useAuth } from '../../components/Auth/AuthProvider';
import { useHistory } from 'react-router-dom';
import '../../components/Auth/Auth.css';

const VerifyEmailPage = () => {
  const { user } = useAuth();
  const history = useHistory();

  const handleResendEmail = () => {
    // Logic to resend verification email
    console.log('Resend verification email');
    // In a real implementation, you would call an API to resend the verification email
  };

  const handleGoToDashboard = () => {
    history.push('/dashboard');
  };

  return (
    <div className="page-container">
      <div className="auth-page">
        <div className="auth-content">
          <div className="auth-form">
            <h2>Email Verification Required</h2>

            <div className="verification-message">
              <p>
                We've sent a verification email to <strong>{user?.email || 'your email address'}</strong>.
                Please check your inbox and click the verification link to activate your account.
              </p>
            </div>

            <div className="verification-actions">
              <button
                className="auth-button"
                onClick={handleResendEmail}
              >
                Resend Verification Email
              </button>

              <button
                className="secondary-button"
                onClick={handleGoToDashboard}
              >
                Skip for Now
              </button>
            </div>

            <div className="auth-links">
              <p>
                Already verified? <a href="/auth/signin">Sign in</a>
              </p>
            </div>
          </div>
        </div>

        <div className="auth-illustration">
          <div className="auth-branding">
            <h1>Almost There!</h1>
            <p>Verify your email to access all features</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default VerifyEmailPage;