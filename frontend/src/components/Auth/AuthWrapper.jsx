import React from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import authService from '../../services/authService';

const AuthWrapper = ({ children, requireAuth = true }) => {
  const location = useLocation();
  const isAuthenticated = authService.isAuthenticated();

  // If the route requires authentication but user is not authenticated
  if (requireAuth && !isAuthenticated) {
    // Redirect to login page and save the attempted route
    return <Navigate to="/login" state={{ from: location }} replace />;
  }

  // If the route doesn't require authentication but user is already authenticated
  if (!requireAuth && isAuthenticated) {
    // Redirect to home page
    return <Navigate to="/" replace />;
  }

  // Otherwise, render the children
  return children;
};

export default AuthWrapper;