import React, { createContext, useContext, useReducer, useEffect } from 'react';
import { authApi } from '../services/authApi';

// Create Auth Context
const AuthContext = createContext();

// Initial state
const initialState = {
  user: null,
  profile: null,
  token: null,
  isAuthenticated: false,
  isLoading: true,
  error: null,
};

// Reducer to handle auth state
const authReducer = (state, action) => {
  switch (action.type) {
    case 'AUTH_START':
      return {
        ...state,
        isLoading: true,
        error: null,
      };
    case 'AUTH_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        profile: action.payload.profile,
        token: action.payload.token,
        isAuthenticated: true,
        isLoading: false,
        error: null,
      };
    case 'AUTH_ERROR':
      return {
        ...state,
        isLoading: false,
        error: action.payload,
      };
    case 'SET_USER':
      return {
        ...state,
        user: action.payload.user,
        profile: action.payload.profile,
        token: action.payload.token,
        isAuthenticated: !!action.payload.token,
        isLoading: false,
      };
    case 'LOGOUT':
      return {
        ...state,
        user: null,
        profile: null,
        token: null,
        isAuthenticated: false,
        isLoading: false,
      };
    case 'CLEAR_ERROR':
      return {
        ...state,
        error: null,
      };
    default:
      return state;
  }
};

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  // Check if user is logged in on initial load
  useEffect(() => {
    const token = localStorage.getItem('token');
    if (token) {
      // Verify token and get user info
      verifyTokenAndSetUser(token);
    } else {
      dispatch({ type: 'AUTH_SUCCESS' }); // Set loading to false
    }
  }, []);

  const verifyTokenAndSetUser = async (token) => {
    try {
      const response = await authApi.getCurrentUser();
      dispatch({
        type: 'SET_USER',
        payload: {
          user: response.user,
          profile: response.profile,
          token: token,
        },
      });
    } catch (error) {
      // Token is invalid, clear it
      localStorage.removeItem('token');
      dispatch({ type: 'LOGOUT' });
    }
  };

  // Sign up function
  const signUp = async (email, password, profileData) => {
    dispatch({ type: 'AUTH_START' });
    try {
      const response = await authApi.signUp(email, password, profileData);
      const { user, profile, access_token } = response;

      // Store token in localStorage
      localStorage.setItem('token', access_token);

      dispatch({
        type: 'AUTH_SUCCESS',
        payload: {
          user,
          profile,
          token: access_token,
        },
      });

      return { success: true };
    } catch (error) {
      const errorMessage = error.response?.data?.detail || 'Sign up failed';
      dispatch({
        type: 'AUTH_ERROR',
        payload: errorMessage,
      });
      return { success: false, error: errorMessage };
    }
  };

  // Sign in function
  const signIn = async (email, password) => {
    dispatch({ type: 'AUTH_START' });
    try {
      const response = await authApi.signIn(email, password);
      const { user, profile, access_token } = response;

      // Store token in localStorage
      localStorage.setItem('token', access_token);

      dispatch({
        type: 'AUTH_SUCCESS',
        payload: {
          user,
          profile,
          token: access_token,
        },
      });

      return { success: true };
    } catch (error) {
      const errorMessage = error.response?.data?.detail || 'Sign in failed';
      dispatch({
        type: 'AUTH_ERROR',
        payload: errorMessage,
      });
      return { success: false, error: errorMessage };
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
      dispatch({ type: 'LOGOUT' });
    }
  };

  // Get current user
  const getCurrentUser = async () => {
    try {
      const response = await authApi.getCurrentUser();
      dispatch({
        type: 'SET_USER',
        payload: {
          user: response.user,
          profile: response.profile,
          token: state.token,
        },
      });
      return response;
    } catch (error) {
      console.error('Error getting current user:', error);
      throw error;
    }
  };

  // Update profile
  const updateProfile = async (profileData) => {
    try {
      const response = await authApi.updateProfile(profileData);
      dispatch({
        type: 'SET_USER',
        payload: {
          user: state.user,
          profile: response.profile,
          token: state.token,
        },
      });
      return response;
    } catch (error) {
      console.error('Error updating profile:', error);
      throw error;
    }
  };

  // Clear error
  const clearError = () => {
    dispatch({ type: 'CLEAR_ERROR' });
  };

  const value = {
    ...state,
    signUp,
    signIn,
    signOut,
    getCurrentUser,
    updateProfile,
    clearError,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default AuthContext;