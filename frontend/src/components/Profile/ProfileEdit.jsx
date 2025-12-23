import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import authService from '../../services/authService';

const ProfileEdit = () => {
  const [formData, setFormData] = useState({
    name: '',
    avatar_url: ''
  });
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState(null);
  const [message, setMessage] = useState('');

  const navigate = useNavigate();

  useEffect(() => {
    const fetchUserProfile = async () => {
      try {
        const result = await authService.getCurrentUser();
        if (result.success) {
          setFormData({
            name: result.user.name || '',
            avatar_url: result.user.avatar_url || ''
          });
        } else {
          setError(result.error || 'Failed to load user profile');
        }
      } catch (err) {
        setError(err.message || 'An error occurred while loading profile');
      } finally {
        setLoading(false);
      }
    };

    if (authService.isAuthenticated()) {
      fetchUserProfile();
    } else {
      navigate('/login');
    }
  }, [navigate]);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setSaving(true);
    setMessage('');
    setError('');

    try {
      const result = await authService.updateProfile(formData);

      if (result.success) {
        setMessage('Profile updated successfully!');
        setTimeout(() => {
          navigate('/profile');
        }, 1500);
      } else {
        setError(result.error || 'Failed to update profile');
      }
    } catch (err) {
      setError(err.message || 'An error occurred while updating profile');
    } finally {
      setSaving(false);
    }
  };

  if (loading) {
    return <div className="profile-edit">Loading profile...</div>;
  }

  if (error && !loading) {
    return <div className="profile-edit error">Error: {error}</div>;
  }

  return (
    <div className="profile-edit">
      <h2>Edit Profile</h2>

      {message && <div className="message success">{message}</div>}
      {error && <div className="message error">{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="name">Full Name</label>
          <input
            type="text"
            id="name"
            name="name"
            value={formData.name}
            onChange={handleChange}
            placeholder="Enter your full name"
          />
        </div>

        <div className="form-group">
          <label htmlFor="avatar_url">Avatar URL</label>
          <input
            type="text"
            id="avatar_url"
            name="avatar_url"
            value={formData.avatar_url}
            onChange={handleChange}
            placeholder="Enter URL to your avatar image"
          />
        </div>

        <div className="form-actions">
          <button type="button" onClick={() => navigate('/profile')} className="cancel-btn">
            Cancel
          </button>
          <button type="submit" disabled={saving} className="save-btn">
            {saving ? 'Saving...' : 'Save Changes'}
          </button>
        </div>
      </form>
    </div>
  );
};

export default ProfileEdit;