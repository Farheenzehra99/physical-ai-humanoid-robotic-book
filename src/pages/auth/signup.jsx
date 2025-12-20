import React from 'react';
import SignupForm from '../../components/Auth/SignupForm';
import '../../components/Auth/Auth.css'; // Page-specific styles

const SignupPage = () => {
  return (
    <div className="page-container">
      <div className="auth-page">
        <div className="auth-content">
          <SignupForm />
        </div>
        {/* <div className="auth-illustration"> */}
          {/* Optional illustration or branding */}
          {/* <div className="auth-branding"> */}
            {/* <h1>Welcome to AI Robotics Book</h1>
            <p>Join our community to access personalized AI and robotics content</p>
          </div>
        </div> */}
      </div>
    </div>
  );
};

export default SignupPage;