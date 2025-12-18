/**
 * Custom Navbar Content component with auth buttons.
 *
 * Wraps the original navbar content and adds authentication buttons.
 * Preserves the default navbar items (logo, Textbook link, GitHub, etc.)
 * and adds Sign Up/Login or User/Logout buttons.
 */

import React, { useState } from 'react';
import OriginalNavbarContent from '@theme-original/Navbar/Content';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../../../context/AuthContext';
import AuthModal from '../../../components/AuthModal';
import Link from '@docusaurus/Link';

export default function NavbarContent(props): JSX.Element {
  const { user, isAuthenticated, logout } = useAuth();
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [authMode, setAuthMode] = useState<'login' | 'signup'>('login');
  const homeUrl = useBaseUrl('/');

  const handleOpenLogin = () => {
    setAuthMode('login');
    setIsAuthModalOpen(true);
  };

  const handleOpenSignup = () => {
    setAuthMode('signup');
    setIsAuthModalOpen(true);
  };

  const handleCloseModal = () => {
    setIsAuthModalOpen(false);
  };

  const handleLogout = () => {
    logout();
    // Force redirect to homepage after logout
    window.location.href = homeUrl;
  };

  return (
    <>
      {/* Render original navbar content (logo, links, etc.) */}
      <OriginalNavbarContent {...props} />

      {/* Add auth buttons to the right */}
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem', marginLeft: '1rem' }}>
        {isAuthenticated && user ? (
          <>
            <Link
              to="/profile"
              style={{
                padding: '0.4rem 0.8rem',
                borderRadius: '6px',
                textDecoration: 'none',
                fontWeight: 500,
                fontSize: '0.9rem',
                transition: 'background-color 0.2s',
                color: 'var(--ifm-navbar-link-color)',
              }}
            >
              {user.email}
            </Link>
            <button
              onClick={handleLogout}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: 'var(--ifm-color-emphasis-200)',
                border: 'none',
                borderRadius: '6px',
                cursor: 'pointer',
                fontWeight: 600,
                fontSize: '0.9rem',
                transition: 'all 0.2s',
              }}
            >
              Logout
            </button>
          </>
        ) : (
          <>
            <button
              onClick={handleOpenLogin}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: 'transparent',
                border: '2px solid var(--ifm-color-primary)',
                color: 'var(--ifm-color-primary)',
                borderRadius: '6px',
                cursor: 'pointer',
                fontWeight: 600,
                fontSize: '0.9rem',
                transition: 'all 0.2s',
              }}
            >
              Login
            </button>
            <button
              onClick={handleOpenSignup}
              style={{
                padding: '0.4rem 1rem',
                backgroundColor: 'var(--ifm-color-primary)',
                color: 'white',
                border: 'none',
                borderRadius: '6px',
                cursor: 'pointer',
                fontWeight: 600,
                fontSize: '0.9rem',
                transition: 'all 0.2s',
              }}
            >
              Sign Up
            </button>
          </>
        )}
      </div>

      <AuthModal
        isOpen={isAuthModalOpen}
        onClose={handleCloseModal}
        initialMode={authMode}
      />
    </>
  );
}
