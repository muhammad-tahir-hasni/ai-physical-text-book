/**
 * ProtectedRoute wrapper component for protecting authenticated routes.
 *
 * Redirects to login if user is not authenticated.
 * Shows loading state while checking authentication.
 */

import React from 'react';
import { Redirect } from '@docusaurus/router';
import { useAuth } from '../context/AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
  redirectTo?: string;
}

export default function ProtectedRoute({
  children,
  redirectTo = '/',
}: ProtectedRouteProps) {
  const { isAuthenticated, isLoading } = useAuth();

  // Show loading state while checking auth
  if (isLoading) {
    return (
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '50vh',
      }}>
        <div>Loading...</div>
      </div>
    );
  }

  // Redirect to home/login if not authenticated
  if (!isAuthenticated) {
    return <Redirect to={redirectTo} />;
  }

  // Render protected content
  return <>{children}</>;
}
