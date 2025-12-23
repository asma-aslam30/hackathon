import React, { useState } from 'react';
import Layout from '@theme/Layout';

const API_BASE_URL = typeof window !== 'undefined'
  ? (window.__BACKEND_URL__ || 'http://localhost:8000')
  : 'http://localhost:8000';

export default function SignupPage() {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    // Validate passwords match
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    // Validate password length
    if (password.length < 6) {
      setError('Password must be at least 6 characters');
      return;
    }

    setLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/api/v1/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ name, email, password }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Store the token
        localStorage.setItem('session_token', data.session_token);
        localStorage.setItem('user', JSON.stringify(data.user));

        // Redirect to home
        window.location.href = '/hackathon/';
      } else {
        setError(data.detail || 'Registration failed. Please try again.');
      }
    } catch (err) {
      setError('Network error. Please try again.');
      console.error('Signup error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create a new account">
      <div style={styles.container}>
        <div style={styles.card}>
          <h1 style={styles.title}>Create Account</h1>

          {error && (
            <div style={styles.error}>
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit} style={styles.form}>
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
                placeholder="Create a password"
                required
                minLength={6}
                style={styles.input}
              />
            </div>

            <div style={styles.inputGroup}>
              <label style={styles.label}>Confirm Password</label>
              <input
                type="password"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                placeholder="Confirm your password"
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
              {loading ? 'Creating account...' : 'Sign Up'}
            </button>
          </form>

          <p style={styles.footer}>
            Already have an account?{' '}
            <a href="/hackathon/login" style={styles.link}>
              Login
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
