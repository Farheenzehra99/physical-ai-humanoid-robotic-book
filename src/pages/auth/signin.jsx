import React from 'react';
import SigninForm from '../../components/Auth/SigninForm';
import '../../components/Auth/Auth.css'; // Page-specific styles

const SigninPage = () => {
  return (
    <div className="page-container">
      <div className="auth-page">
        <div className="auth-content">
          <SigninForm />
        </div>
        {/* <div className="auth-illustration"> */}
          {/* Optional illustration or branding */}
          {/* <div className="auth-branding">
            <h1>Welcome Back</h1>
            <p>Continue your AI and robotics learning journey</p>
          </div>
        </div> */}
      </div>
    </div>
  );
};

export default SigninPage;