import { useState, useEffect } from 'react';
import authApi from '../services/authApi';

// Simple auth hook for components that need auth functionality
export const useAuth = () => {
  const [user, setUser] = useState(null);
  const [profile, setProfile] = useState(null);
  const [token, setToken] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  // Check if user is logged in on initial load
  useEffect(() => {
    const token = localStorage.getItem('token');
    if (token) {
      // Verify token and get user info
      verifyTokenAndSetUser(token);
    } else {
      setIsLoading(false);
    }
  }, []);

  const verifyTokenAndSetUser = async (token) => {
    try {
      const response = await authApi.getCurrentUser();
      setUser(response.user);
      setProfile(response.profile);
      setToken(token);
      setIsAuthenticated(true);
    } catch (error) {
      // Token is invalid, clear it
      localStorage.removeItem('token');
      setUser(null);
      setProfile(null);
      setToken(null);
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  };

  // Sign up function
  const signUp = async (email, password, profileData) => {
    setIsLoading(true);
    setError(null);
    try {
      const response = await authApi.signUp(email, password, profileData);
      const { user, profile, access_token } = response;

      // Store token in localStorage
      localStorage.setItem('token', access_token);

      setUser(user);
      setProfile(profile);
      setToken(access_token);
      setIsAuthenticated(true);

      return { success: true };
    } catch (error) {
      const errorMessage = error.response?.data?.detail || 'Sign up failed';
      setError(errorMessage);
      return { success: false, error: errorMessage };
    } finally {
      setIsLoading(false);
    }
  };

  // Sign in function
  const signIn = async (email, password) => {
    setIsLoading(true);
    setError(null);
    try {
      const response = await authApi.signIn(email, password);
      const { user, profile, access_token } = response;

      // Store token in localStorage
      localStorage.setItem('token', access_token);

      setUser(user);
      setProfile(profile);
      setToken(access_token);
      setIsAuthenticated(true);

      return { success: true };
    } catch (error) {
      const errorMessage = error.response?.data?.detail || 'Sign in failed';
      setError(errorMessage);
      return { success: false, error: errorMessage };
    } finally {
      setIsLoading(false);
    }
  };

  // Sign out function
  const signOut = async () => {
    try {
      await authApi.signOut();
    } catch (error) {
      // Even if sign out fails on the server, clear local data
      console.error('Sign out error:', error);
    } finally {
      // Clear token from localStorage
      localStorage.removeItem('token');
      setUser(null);
      setProfile(null);
      setToken(null);
      setIsAuthenticated(false);
    }
  };

  // Get current user
  const getCurrentUser = async () => {
    try {
      const response = await authApi.getCurrentUser();
      setUser(response.user);
      setProfile(response.profile);
      return response;
    } catch (error) {
      console.error('Error getting current user:', error);
      throw error;
    }
  };

  return {
    user,
    profile,
    token,
    isAuthenticated,
    isLoading,
    error,
    signUp,
    signIn,
    signOut,
    getCurrentUser,
  };
};