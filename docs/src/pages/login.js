import React, { useState } from 'react';
import Layout from '@theme/Layout';
import API_CONFIG from '../config/api';

const API_BASE_URL = API_CONFIG.BACKEND_URL;

export default function LoginPage() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Store the token
        localStorage.setItem('session_token', data.session_token);
        localStorage.setItem('user', JSON.stringify(data.user));

        // Redirect to home or previous page
        window.location.href = '/hackathon/';
      } else {
        setError(data.detail || 'Login failed. Please check your credentials.');
      }
    } catch (err) {
      setError('Network error. Please try again.');
      console.error('Login error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Login" description="Login to your account">
      <div style={styles.container}>
        <div style={styles.card}>
          <h1 style={styles.title}>Login</h1>

          {error && (
            <div style={styles.error}>
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit} style={styles.form}>
            <div style={styles.inputGroup}>
              <label style={styles.label}>Email</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="Enter your email"
                required
                style={styles.input}
              />
            </div>

            <div style={styles.inputGroup}>
              <label style={styles.label}>Password</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="Enter your password"
                required
                style={styles.input}
              />
            </div>

            <button
              type="submit"
              disabled={loading}
              style={{
                ...styles.button,
                opacity: loading ? 0.7 : 1,
              }}
            >
              {loading ? 'Logging in...' : 'Login'}
            </button>
          </form>

          <p style={styles.footer}>
            Don't have an account?{' '}
            <a href="/hackathon/signup" style={styles.link}>
              Sign up
            </a>
          </p>
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
    maxWidth: '400px',
    boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
  },
  title: {
    textAlign: 'center',
    marginBottom: '1.5rem',
    color: 'var(--ifm-heading-color)',
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
  button: {
    padding: '0.75rem',
    borderRadius: '8px',
    border: 'none',
    backgroundColor: 'var(--ifm-color-primary)',
    color: 'white',
    fontSize: '1rem',
    fontWeight: '600',
    cursor: 'pointer',
    marginTop: '0.5rem',
  },
  error: {
    backgroundColor: '#fee2e2',
    color: '#dc2626',
    padding: '0.75rem',
    borderRadius: '8px',
    marginBottom: '1rem',
    textAlign: 'center',
  },
  footer: {
    textAlign: 'center',
    marginTop: '1.5rem',
    color: 'var(--ifm-font-color-secondary)',
  },
  link: {
    color: 'var(--ifm-color-primary)',
    textDecoration: 'none',
    fontWeight: '500',
  },
};
