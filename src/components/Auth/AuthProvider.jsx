import React, { createContext, useContext, useReducer, useEffect } from 'react';

// Create Auth Context
const AuthContext = createContext();

// Get API base URL
const getApiUrl = () => {
  if (typeof window !== 'undefined' && window.__DOCUSAURUS__) {
    return window.__DOCUSAURUS__.siteConfig.customFields?.chatApiUrl || 'http://localhost:8000';
  }
  return 'http://localhost:8000';
};

// Auth Reducer
const authReducer = (state, action) => {
  switch (action.type) {
    case 'LOGIN_START':
      return {
        ...state,
        loading: true,
        error: null
      };
    case 'LOGIN_SUCCESS':
      return {
        ...state,
        loading: false,
        isAuthenticated: true,
        user: action.payload.user,
        token: action.payload.token,
        error: null
      };
    case 'LOGIN_FAILURE':
      return {
        ...state,
        loading: false,
        isAuthenticated: false,
        user: null,
        token: null,
        error: action.payload
      };
    case 'LOGOUT':
      return {
        ...state,
        loading: false,
        isAuthenticated: false,
        user: null,
        token: null,
        error: null
      };
    case 'SET_USER_PROFILE':
      return {
        ...state,
        user: {
          ...state.user,
          profile: action.payload
        }
      };
    case 'UPDATE_USER_PROFILE':
      return {
        ...state,
        user: {
          ...state.user,
          profile: {
            ...state.user.profile,
            ...action.payload
          }
        }
      };
    case 'CLEAR_ERROR':
      return {
        ...state,
        error: null
      };
    default:
      return state;
  }
};

// Initial State
const initialState = {
  loading: false,
  isAuthenticated: false,
  user: null,
  token: null,
  error: null
};

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  // Check for existing session on initial load
  useEffect(() => {
    const token = localStorage.getItem('authToken');
    const user = localStorage.getItem('user');

    if (token && user) {
      try {
        const parsedUser = JSON.parse(user);
        dispatch({
          type: 'LOGIN_SUCCESS',
          payload: {
            user: parsedUser,
            token: token
          }
        });
      } catch (error) {
        console.error('Failed to parse stored user data:', error);
        logout();
      }
    }
  }, []);

  // Login function
  const login = async (email, password) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/api/v1/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',  // Include cookies for cross-origin requests
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Store token if provided
        if (data.session_token) {
          localStorage.setItem('authToken', data.session_token);
        }

        // Store user info
        const user = data.user || {
          id: data.user_id,
          email: data.email,
          first_name: data.first_name,
          last_name: data.last_name
        };
        localStorage.setItem('user', JSON.stringify(user));

        dispatch({
          type: 'LOGIN_SUCCESS',
          payload: {
            user: user,
            token: data.session_token || null
          }
        });

        return { success: true, redirectTo: data.redirectTo || '/dashboard' };
      } else {
        const errorMessage = data.detail || data.error || 'Login failed';
        dispatch({
          type: 'LOGIN_FAILURE',
          payload: errorMessage
        });
        return { success: false, error: errorMessage };
      }
    } catch (error) {
      dispatch({
        type: 'LOGIN_FAILURE',
        payload: error.message || 'Network error'
      });
      return { success: false, error: error.message || 'Network error' };
    }
  };

  // Signup function
  const signup = async (userData) => {
    dispatch({ type: 'LOGIN_START' });

    try {
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/api/v1/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',  // Include cookies for cross-origin requests
        body: JSON.stringify({
          email: userData.email,
          password: userData.password,
          first_name: userData.first_name,
          last_name: userData.last_name,
          programming_level: userData.programming_level,
          programming_languages: userData.programming_languages || [],
          ai_knowledge_level: userData.ai_knowledge_level,
          hardware_experience: userData.hardware_experience,
          learning_style: userData.learning_style
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Store token if provided
        if (data.session_token) {
          localStorage.setItem('authToken', data.session_token);
        }

        // Store user info
        const user = {
          id: data.user_id,
          email: data.email,
          first_name: data.first_name,
          last_name: data.last_name
        };
        localStorage.setItem('user', JSON.stringify(user));

        dispatch({
          type: 'LOGIN_SUCCESS',
          payload: {
            user: user,
            token: data.session_token || null
          }
        });

        return { success: true, requiresEmailVerification: data.requires_email_verification || true };
      } else {
        const errorMessage = data.detail || data.error || 'Signup failed';
        dispatch({
          type: 'LOGIN_FAILURE',
          payload: errorMessage
        });
        return { success: false, error: errorMessage };
      }
    } catch (error) {
      dispatch({
        type: 'LOGIN_FAILURE',
        payload: error.message || 'Network error'
      });
      return { success: false, error: error.message || 'Network error' };
    }
  };

  // Logout function
  const logout = async () => {
    try {
      // Call backend logout endpoint
      const token = localStorage.getItem('authToken');
      if (token) {
        const apiUrl = getApiUrl();
        await fetch(`${apiUrl}/api/v1/auth/signout`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `Bearer ${token}`
          },
          credentials: 'include'  // Include cookies for cross-origin requests
        });
      }
    } catch (error) {
      console.error('Logout error:', error);
    }

    // Clear localStorage
    localStorage.removeItem('authToken');
    localStorage.removeItem('user');

    dispatch({ type: 'LOGOUT' });
  };

  // Get user profile
  const getUserProfile = async () => {
    if (!state.token) {
      return null;
    }

    try {
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/api/v1/auth/me`, {
        headers: {
          'Authorization': `Bearer ${state.token}`,
        },
        credentials: 'include',  // Include cookies for cross-origin requests
      });

      if (response.ok) {
        const data = await response.json();
        dispatch({
          type: 'SET_USER_PROFILE',
          payload: data.profile
        });
        return data.profile;
      } else {
        console.error('Failed to fetch user profile');
        return null;
      }
    } catch (error) {
      console.error('Error fetching user profile:', error);
      return null;
    }
  };

  // Update user profile
  const updateUserProfile = async (profileData) => {
    if (!state.token) {
      return false;
    }

    try {
      const apiUrl = getApiUrl();
      const response = await fetch(`${apiUrl}/api/v1/auth/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${state.token}`,
        },
        credentials: 'include',  // Include cookies for cross-origin requests
        body: JSON.stringify(profileData),
      });

      if (response.ok) {
        const data = await response.json();
        dispatch({
          type: 'UPDATE_USER_PROFILE',
          payload: data.profile
        });
        return true;
      } else {
        console.error('Failed to update user profile');
        return false;
      }
    } catch (error) {
      console.error('Error updating user profile:', error);
      return false;
    }
  };

  // Clear error
  const clearError = () => {
    dispatch({ type: 'CLEAR_ERROR' });
  };

  const value = {
    ...state,
    login,
    signup,
    logout,
    getUserProfile,
    updateUserProfile,
    clearError
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