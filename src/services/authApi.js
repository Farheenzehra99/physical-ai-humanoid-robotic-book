import axios from 'axios';

// Get API base URL from Docusaurus config or default to Hugging Face Space
const getApiBaseUrl = () => {
  // Force Hugging Face Space URL for production
  return 'https://farheenzehra99-ai-book.hf.space';
};

const API_BASE_URL = getApiBaseUrl();

// Create axios instance with defaults
const api = axios.create({
  baseURL: API_BASE_URL,
  timeout: 10000,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Request interceptor to add token to requests
api.interceptors.request.use(
  (config) => {
    const token = localStorage.getItem('authToken');
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Response interceptor to handle token expiration
api.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response && error.response.status === 401) {
      // Token might be expired, remove it
      localStorage.removeItem('authToken');
      localStorage.removeItem('user');
      // Optionally redirect to login page
      // window.location.href = '/auth/signin';
    }
    return Promise.reject(error);
  }
);

// Auth API methods
const authApi = {
  // Sign up
  signUp: async (email, password, profileData) => {
    const response = await api.post('/auth/signup', {
      email,
      password,
      profile: profileData,
    });
    return response.data;
  },

  // Sign in
  signIn: async (email, password) => {
    const response = await api.post('/auth/signin', {
      email,
      password,
    });
    return response.data;
  },

  // Sign out (optional server-side sign out)
  signOut: async () => {
    // Currently just returns success since we're using JWT tokens
    // In a real implementation, you might want to add the token to a blacklist
    return { success: true };
  },

  // Get current user
  getCurrentUser: async () => {
    const response = await api.get('/auth/me');
    return response.data;
  },

  // Update user profile
  updateProfile: async (profileData) => {
    // Note: In our current implementation, we don't have a separate endpoint for updating profile
    // This would need to be implemented on the backend if needed
    throw new Error('Update profile not implemented yet');
  },
};

export default authApi;