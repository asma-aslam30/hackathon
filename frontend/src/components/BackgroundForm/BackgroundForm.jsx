import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import authService from '../../services/authService';
import { validateForm, backgroundValidationSchema } from '../../utils/validation';
import BackgroundFields from './BackgroundFields';

const BackgroundForm = () => {
  const [formData, setFormData] = useState({
    software_tools: [],
    hardware_setup: { os: '', cpu: '', ram: '', gpu: '' },
    programming_languages: [],
    technical_preferences: { ide_preference: '', work_style: '', preferred_environment: '' },
    experience_level: '',
    primary_domain: ''
  });
  const [errors, setErrors] = useState({});
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  const navigate = useNavigate();

  // Check authentication status on component mount
  useEffect(() => {
    const checkAuth = async () => {
      if (!authService.isAuthenticated()) {
        navigate('/login');
      } else {
        setIsAuthenticated(true);
      }
    };

    checkAuth();
  }, [navigate]);

  const handleChange = (field, value) => {
    setFormData({
      ...formData,
      [field]: value
    });

    // Clear error when user updates the field
    if (errors[field]) {
      setErrors({
        ...errors,
        [field]: ''
      });
    }
  };

  const handleNestedChange = (parentField, childField, value) => {
    setFormData({
      ...formData,
      [parentField]: {
        ...formData[parentField],
        [childField]: value
      }
    });

    // Clear error when user updates the field
    if (errors[`${parentField}.${childField}`]) {
      setErrors({
        ...errors,
        [`${parentField}.${childField}`]: ''
      });
    }
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Validate form
    const { isValid, errors: formErrors } = validateForm(formData, backgroundValidationSchema);
    setErrors(formErrors);

    if (!isValid) {
      setMessage('Please fix the errors in the form');
      return;
    }

    setLoading(true);
    setMessage('');

    try {
      // We'll need to create an API service for background data
      // For now, let's create a temporary implementation
      const response = await fetch(`${process.env.REACT_APP_API_URL || 'http://localhost:8000/api'}/background/me`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${authService.getToken()}`
        },
        body: JSON.stringify({
          software_tools: formData.software_tools,
          hardware_setup: formData.hardware_setup,
          programming_languages: formData.programming_languages,
          technical_preferences: formData.technical_preferences,
          experience_level: formData.experience_level,
          primary_domain: formData.primary_domain
        })
      });

      const result = await response.json();

      if (result.success) {
        setMessage('Background information saved successfully! Redirecting...');
        setTimeout(() => {
          navigate('/'); // Redirect to home after successful submission
        }, 1500);
      } else {
        setMessage(result.error || 'Failed to save background information');
      }
    } catch (error) {
      setMessage(error.message || 'An error occurred while saving background information');
    } finally {
      setLoading(false);
    }
  };

  if (!isAuthenticated) {
    return null; // Render nothing while checking authentication
  }

  return (
    <div className="background-form-container">
      <div className="background-form">
        <h2>Tell Us About Your Background</h2>
        <p>Help us personalize your experience by sharing information about your technical background.</p>

        {message && (
          <div className={`message ${message.includes('successful') ? 'success' : 'error'}`}>
            {message}
          </div>
        )}

        <form onSubmit={handleSubmit}>
          <BackgroundFields
            formData={formData}
            errors={errors}
            handleChange={handleChange}
            handleNestedChange={handleNestedChange}
          />

          <button type="submit" disabled={loading} className="submit-btn">
            {loading ? 'Saving...' : 'Save Background Information'}
          </button>
        </form>
      </div>
    </div>
  );
};

export default BackgroundForm;