import React from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import { useAuth } from './AuthProvider';

const ProtectedRoute = ({ children, allowedRoles = [], allowedPermissions = [] }) => {
  const { isAuthenticated, loading, user } = useAuth();
  const location = useLocation();

  // Show loading state while checking authentication
  if (loading) {
    return (
      <div className="loading-container">
        <div className="loading-spinner">Loading...</div>
      </div>
    );
  }

  // If not authenticated, redirect to login with return URL
  if (!isAuthenticated) {
    return <Navigate to={`/auth/signin?redirect=${location.pathname}`} replace />;
  }

  // Check role-based access if roles are specified
  if (allowedRoles.length > 0) {
    const hasRole = allowedRoles.some(role =>
      user?.roles?.includes(role) || user?.profile?.role === role
    );

    if (!hasRole) {
      return (
        <div className="access-denied">
          <h2>Access Denied</h2>
          <p>You don't have the required role to access this page.</p>
          <button onClick={() => window.history.back()}>Go Back</button>
        </div>
      );
    }
  }

  // Check permission-based access if permissions are specified
  if (allowedPermissions.length > 0) {
    const hasPermission = allowedPermissions.some(permission =>
      user?.permissions?.includes(permission) ||
      user?.profile?.permissions?.includes(permission)
    );

    if (!hasPermission) {
      return (
        <div className="access-denied">
          <h2>Access Denied</h2>
          <p>You don't have the required permissions to access this page.</p>
          <button onClick={() => window.history.back()}>Go Back</button>
        </div>
      );
    }
  }

  // User is authenticated and has required roles/permissions
  return children;
};

export default ProtectedRoute;