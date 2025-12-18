/**
 * API client with automatic token refresh on 401 errors.
 *
 * Features:
 * - Automatic token injection from localStorage
 * - Intercept 401 errors and refresh token
 * - Retry original request with new token
 * - Logout on refresh failure
 */

// Get API URL from window object (set by Root component) or fallback to localhost
function getApiBaseUrl(): string {
  if (typeof window !== 'undefined' && (window as any).__DOCUSAURUS_API_BASE_URL__) {
    return (window as any).__DOCUSAURUS_API_BASE_URL__;
  }
  return 'http://localhost:8000';
}

const API_BASE_URL = getApiBaseUrl();

interface RequestOptions extends RequestInit {
  skipAuthRefresh?: boolean;
}

/**
 * Make authenticated API request with automatic token refresh.
 *
 * @param endpoint - API endpoint (e.g., '/api/v1/user/profile')
 * @param options - Fetch options
 * @returns Response promise
 */
export async function apiRequest(
  endpoint: string,
  options: RequestOptions = {}
): Promise<Response> {
  const { skipAuthRefresh = false, ...fetchOptions } = options;

  // Inject access token
  const accessToken = localStorage.getItem('access_token');
  const headers = new Headers(fetchOptions.headers);

  if (accessToken) {
    headers.set('Authorization', `Bearer ${accessToken}`);
  }

  // Include cookies for httpOnly token
  const config: RequestInit = {
    ...fetchOptions,
    headers,
    credentials: 'include',
  };

  const url = `${API_BASE_URL}${endpoint}`;
  let response = await fetch(url, config);

  // Handle 401 by refreshing token and retrying
  if (response.status === 401 && !skipAuthRefresh) {
    const refreshSuccess = await refreshAccessToken();

    if (refreshSuccess) {
      // Retry original request with new token
      const newAccessToken = localStorage.getItem('access_token');
      if (newAccessToken) {
        headers.set('Authorization', `Bearer ${newAccessToken}`);
        config.headers = headers;
        response = await fetch(url, config);
      }
    } else {
      // Refresh failed - redirect to login
      handleAuthFailure();
    }
  }

  return response;
}

/**
 * Refresh access token using refresh token.
 *
 * @returns true if refresh successful, false otherwise
 */
async function refreshAccessToken(): Promise<boolean> {
  try {
    const refreshToken = localStorage.getItem('refresh_token');
    if (!refreshToken) {
      return false;
    }

    const response = await fetch(`${API_BASE_URL}/api/v1/auth/refresh`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include',
      body: JSON.stringify({ refresh_token: refreshToken }),
    });

    if (!response.ok) {
      return false;
    }

    const data = await response.json();

    // Update stored tokens
    localStorage.setItem('access_token', data.access_token);
    localStorage.setItem('refresh_token', data.refresh_token);

    return true;
  } catch (error) {
    console.error('Token refresh failed:', error);
    return false;
  }
}

/**
 * Handle authentication failure by clearing tokens and redirecting.
 */
function handleAuthFailure() {
  // Clear all auth data
  localStorage.removeItem('access_token');
  localStorage.removeItem('refresh_token');
  localStorage.removeItem('user');

  // Redirect to home page
  if (typeof window !== 'undefined') {
    window.location.href = '/';
  }
}

/**
 * Helper function to make GET request.
 */
export async function get(endpoint: string, options: RequestOptions = {}): Promise<Response> {
  return apiRequest(endpoint, { ...options, method: 'GET' });
}

/**
 * Helper function to make POST request.
 */
export async function post(
  endpoint: string,
  data?: any,
  options: RequestOptions = {}
): Promise<Response> {
  return apiRequest(endpoint, {
    ...options,
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      ...options.headers,
    },
    body: data ? JSON.stringify(data) : undefined,
  });
}

/**
 * Helper function to make PATCH request.
 */
export async function patch(
  endpoint: string,
  data?: any,
  options: RequestOptions = {}
): Promise<Response> {
  return apiRequest(endpoint, {
    ...options,
    method: 'PATCH',
    headers: {
      'Content-Type': 'application/json',
      ...options.headers,
    },
    body: data ? JSON.stringify(data) : undefined,
  });
}

/**
 * Helper function to make DELETE request.
 */
export async function del(endpoint: string, options: RequestOptions = {}): Promise<Response> {
  return apiRequest(endpoint, { ...options, method: 'DELETE' });
}
