import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import API_CONFIG from '../config/api';

const API_BASE_URL = API_CONFIG.BACKEND_URL;

export default function ProfilePage() {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [editing, setEditing] = useState(false);
  const [name, setName] = useState('');
  const [message, setMessage] = useState({ type: '', text: '' });

  useEffect(() => {
    checkAuth();
  }, []);

  const checkAuth = () => {
    const token = localStorage.getItem('session_token');
    const userData = localStorage.getItem('user');

    if (!token || !userData) {
      window.location.href = '/hackathon/login';
      return;
    }

    try {
      const parsedUser = JSON.parse(userData);
      setUser(parsedUser);
      setName(parsedUser.name);
    } catch (e) {
      window.location.href = '/hackathon/login';
    }
    setLoading(false);
  };

  const handleUpdateProfile = async (e) => {
    e.preventDefault();
    setMessage({ type: '', text: '' });

    const token = localStorage.getItem('session_token');
    if (!token) {
      window.location.href = '/hackathon/login';
      return;
    }

    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({ name }),
      });

      if (response.ok) {
        const updatedUser = { ...user, name };
        setUser(updatedUser);
        localStorage.setItem('user', JSON.stringify(updatedUser));
        setMessage({ type: 'success', text: 'Profile updated successfully!' });
        setEditing(false);
      } else {
        const data = await response.json();
        setMessage({ type: 'error', text: data.detail || 'Failed to update profile' });
      }
    } catch (err) {
      setMessage({ type: 'error', text: 'Network error. Please try again.' });
    }
  };

  const handleLogout = () => {
    localStorage.removeItem('session_token');
    localStorage.removeItem('user');
    window.location.href = '/hackathon/';
  };

  if (loading) {
    return (
      <Layout title="Profile">
        <div style={styles.container}>
          <div style={styles.loading}>Loading...</div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="Manage your profile">
      <div style={styles.container}>
        <div style={styles.card}>
          <div style={styles.header}>
            <div style={styles.avatar}>
              {user?.name?.charAt(0)?.toUpperCase() || 'U'}
            </div>
            <h1 style={styles.title}>My Profile</h1>
          </div>

          {message.text && (
            <div style={{
              ...styles.message,
              backgroundColor: message.type === 'success' ? '#d1fae5' : '#fee2e2',
              color: message.type === 'success' ? '#065f46' : '#dc2626',
            }}>
              {message.text}
            </div>
          )}

          {!editing ? (
            <div style={styles.profileInfo}>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Name</span>
                <span style={styles.infoValue}>{user?.name}</span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Email</span>
                <span style={styles.infoValue}>{user?.email}</span>
              </div>
              <div style={styles.infoRow}>
                <span style={styles.infoLabel}>Member Since</span>
                <span style={styles.infoValue}>
                  {user?.created_at ? new Date(user.created_at).toLocaleDateString() : 'N/A'}
                </span>
              </div>

              <div style={styles.buttonGroup}>
                <button
                  onClick={() => setEditing(true)}
                  style={styles.editButton}
                >
                  Edit Profile
                </button>
                <button
                  onClick={handleLogout}
                  style={styles.logoutButton}
                >
                  Logout
                </button>
              </div>
            </div>
          ) : (
            <form onSubmit={handleUpdateProfile} style={styles.form}>
              <div style={styles.inputGroup}>
                <label style={styles.label}>Name</label>
                <input
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  placeholder="Enter your name"
                  required
                  style={styles.input}
                />
              </div>

              <div style={styles.inputGroup}>
                <label style={styles.label}>Email</label>
                <input
                  type="email"
                  value={user?.email || ''}
                  disabled
                  style={{ ...styles.input, opacity: 0.6, cursor: 'not-allowed' }}
                />
                <small style={styles.helpText}>Email cannot be changed</small>
              </div>

              <div style={styles.buttonGroup}>
                <button type="submit" style={styles.saveButton}>
                  Save Changes
                </button>
                <button
                  type="button"
                  onClick={() => {
                    setEditing(false);
                    setName(user?.name || '');
                  }}
                  style={styles.cancelButton}
                >
                  Cancel
                </button>
              </div>
            </form>
          )}
        </div>
      </div>
    </Layout>
  );
}

const styles = {
  container: {
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    minHeight: '70vh',
    padding: '2rem',
  },
  card: {
    backgroundColor: 'var(--ifm-background-surface-color)',
    borderRadius: '12px',
    padding: '2rem',
    width: '100%',
    maxWidth: '500px',
    boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
  },
  header: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    marginBottom: '2rem',
  },
  avatar: {
    width: '80px',
    height: '80px',
    borderRadius: '50%',
    backgroundColor: 'var(--ifm-color-primary)',
    color: 'white',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    fontSize: '2rem',
    fontWeight: 'bold',
    marginBottom: '1rem',
  },
  title: {
    margin: 0,
    color: 'var(--ifm-heading-color)',
  },
  loading: {
    textAlign: 'center',
    color: 'var(--ifm-font-color-secondary)',
  },
  message: {
    padding: '0.75rem',
    borderRadius: '8px',
    marginBottom: '1rem',
    textAlign: 'center',
  },
  profileInfo: {
    display: 'flex',
    flexDirection: 'column',
    gap: '1rem',
  },
  infoRow: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '0.75rem',
    backgroundColor: 'var(--ifm-background-color)',
    borderRadius: '8px',
    border: '1px solid var(--ifm-color-emphasis-200)',
  },
  infoLabel: {
    fontWeight: '500',
    color: 'var(--ifm-font-color-secondary)',
  },
  infoValue: {
    color: 'var(--ifm-font-color-base)',
  },
  form: {
    display: 'flex',
    flexDirection: 'column',
    gap: '1rem',
  },
  inputGroup: {
    display: 'flex',
    flexDirection: 'column',
    gap: '0.5rem',
  },
  label: {
    fontWeight: '500',
    color: 'var(--ifm-font-color-base)',
  },
  input: {
    padding: '0.75rem',
    borderRadius: '8px',
    border: '1px solid var(--ifm-color-emphasis-300)',
    fontSize: '1rem',
    backgroundColor: 'var(--ifm-background-color)',
    color: 'var(--ifm-font-color-base)',
  },
  helpText: {
    fontSize: '0.75rem',
    color: 'var(--ifm-font-color-secondary)',
  },
  buttonGroup: {
    display: 'flex',
    gap: '1rem',
    marginTop: '1rem',
  },
  editButton: {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: 'none',
    backgroundColor: 'var(--ifm-color-primary)',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600',
    cursor: 'pointer',
  },
  saveButton: {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: 'none',
    backgroundColor: '#10b981',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600',
    cursor: 'pointer',
  },
  cancelButton: {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: '1px solid var(--ifm-color-emphasis-300)',
    backgroundColor: 'transparent',
    color: 'var(--ifm-font-color-base)',
    fontSize: '1rem',
    fontWeight: '600',
    cursor: 'pointer',
  },
  logoutButton: {
    flex: 1,
    padding: '0.75rem',
    borderRadius: '8px',
    border: 'none',
    backgroundColor: '#ef4444',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600',
    cursor: 'pointer',
  },
};
