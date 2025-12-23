import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import authService from '../../services/authService';

const ProfileView = () => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  const navigate = useNavigate();

  useEffect(() => {
    const fetchUserProfile = async () => {
      try {
        const result = await authService.getCurrentUser();
        if (result.success) {
          setUser(result.user);
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

  if (loading) {
    return <div className="profile-view">Loading profile...</div>;
  }

  if (error) {
    return <div className="profile-view error">Error: {error}</div>;
  }

  if (!user) {
    return <div className="profile-view">No user data available</div>;
  }

  return (
    <div className="profile-view">
      <h2>User Profile</h2>

      <div className="profile-info">
        <div className="profile-field">
          <label>Name:</label>
          <span>{user.name || 'Not provided'}</span>
        </div>

        <div className="profile-field">
          <label>Email:</label>
          <span>{user.email}</span>
        </div>

        <div className="profile-field">
          <label>Member Since:</label>
          <span>{user.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}</span>
        </div>

        <div className="profile-field">
          <label>Last Login:</label>
          <span>{user.last_login_at ? new Date(user.last_login_at).toLocaleString() : 'Never'}</span>
        </div>

        <div className="profile-field">
          <label>Status:</label>
          <span className={user.is_active ? 'active' : 'inactive'}>
            {user.is_active ? 'Active' : 'Inactive'}
          </span>
        </div>
      </div>

      <div className="profile-actions">
        <button onClick={() => navigate('/profile/edit')} className="edit-btn">
          Edit Profile
        </button>
        <button onClick={() => navigate('/background')} className="edit-btn">
          Edit Background
        </button>
      </div>
    </div>
  );
};

export default ProfileView;