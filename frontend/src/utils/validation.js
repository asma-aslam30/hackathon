// Validation utilities for forms
import { yup } from 'yup';

// Email validation
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;

// Password validation - at least 8 chars, 1 uppercase, 1 lowercase, 1 number
const passwordRegex = /^(?=.*[a-z])(?=.*[A-Z])(?=.*\d)[a-zA-Z\d@$!%*?&]{8,}$/;

// Validation schemas
export const signupValidationSchema = {
  email: (value) => {
    if (!value) return 'Email is required';
    if (!emailRegex.test(value)) return 'Please enter a valid email address';
    return null;
  },
  password: (value) => {
    if (!value) return 'Password is required';
    if (!passwordRegex.test(value)) return 'Password must be at least 8 characters with uppercase, lowercase, and number';
    return null;
  },
  name: (value) => {
    if (!value) return 'Name is required';
    if (value.length < 2) return 'Name must be at least 2 characters';
    if (value.length > 100) return 'Name must be less than 100 characters';
    return null;
  }
};

export const loginValidationSchema = {
  email: (value) => {
    if (!value) return 'Email is required';
    if (!emailRegex.test(value)) return 'Please enter a valid email address';
    return null;
  },
  password: (value) => {
    if (!value) return 'Password is required';
    return null;
  }
};

export const backgroundValidationSchema = {
  softwareTools: (value) => {
    if (!value || value.length === 0) return 'Please specify at least one software tool you use';
    return null;
  },
  programmingLanguages: (value) => {
    if (!value || value.length === 0) return 'Please specify at least one programming language you know';
    return null;
  },
  experienceLevel: (value) => {
    if (!value) return 'Please specify your experience level';
    const validLevels = ['beginner', 'intermediate', 'advanced', 'expert'];
    if (!validLevels.includes(value)) return 'Please select a valid experience level';
    return null;
  }
};

// Generic validation function
export const validateField = (field, value, schema) => {
  if (schema[field]) {
    return schema[field](value);
  }
  return null;
};

// Validate entire form
export const validateForm = (formData, schema) => {
  const errors = {};
  let isValid = true;

  for (const field in schema) {
    const error = schema[field](formData[field]);
    if (error) {
      errors[field] = error;
      isValid = false;
    }
  }

  return { isValid, errors };
};

// Validate email format
export const isValidEmail = (email) => {
  return emailRegex.test(email);
};

// Validate password strength
export const isValidPassword = (password) => {
  return passwordRegex.test(password);
};

// Validate required field
export const isRequired = (value, fieldName = 'This field') => {
  if (!value || (Array.isArray(value) && value.length === 0)) {
    return `${fieldName} is required`;
  }
  return null;
};