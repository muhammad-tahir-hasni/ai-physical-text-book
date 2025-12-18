/**
 * ErrorBoundary Component
 *
 * Catches React errors and displays a fallback UI instead of crashing the entire app.
 * Logs errors for debugging.
 */

import React, { Component, ErrorInfo, ReactNode } from 'react';

interface Props {
  children: ReactNode;
  fallback?: ReactNode;
}

interface State {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
}

export default class ErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
      errorInfo: null,
    };
  }

  static getDerivedStateFromError(error: Error): Partial<State> {
    // Update state so next render shows fallback UI
    return { hasError: true };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    // Log error to console or error reporting service
    console.error('ErrorBoundary caught an error:', error, errorInfo);

    this.setState({
      error,
      errorInfo,
    });

    // TODO: Send to error reporting service (e.g., Sentry)
    // Sentry.captureException(error, { extra: errorInfo });
  }

  handleReset = (): void => {
    this.setState({
      hasError: false,
      error: null,
      errorInfo: null,
    });
  };

  render(): ReactNode {
    if (this.state.hasError) {
      // Custom fallback UI
      if (this.props.fallback) {
        return this.props.fallback;
      }

      // Default fallback UI
      return (
        <div className="error-boundary">
          <div className="error-boundary-content">
            <h2>⚠️ Something went wrong</h2>
            <p>We're sorry for the inconvenience. The application encountered an unexpected error.</p>

            {process.env.NODE_ENV === 'development' && this.state.error && (
              <details className="error-details">
                <summary>Error Details (Development Only)</summary>
                <div className="error-stack">
                  <h3>{this.state.error.toString()}</h3>
                  <pre>{this.state.errorInfo?.componentStack}</pre>
                </div>
              </details>
            )}

            <div className="error-actions">
              <button onClick={this.handleReset} className="button button--primary">
                Try Again
              </button>
              <button
                onClick={() => window.location.reload()}
                className="button button--secondary"
              >
                Reload Page
              </button>
            </div>
          </div>

          <style>{`
            .error-boundary {
              display: flex;
              justify-content: center;
              align-items: center;
              min-height: 400px;
              padding: 2rem;
              background: var(--ifm-background-surface-color);
              border-radius: 8px;
              margin: 2rem;
            }

            .error-boundary-content {
              max-width: 600px;
              text-align: center;
            }

            .error-boundary-content h2 {
              color: var(--ifm-color-danger);
              margin-bottom: 1rem;
            }

            .error-boundary-content p {
              color: var(--ifm-color-content);
              margin-bottom: 1.5rem;
            }

            .error-details {
              text-align: left;
              margin: 1.5rem 0;
              padding: 1rem;
              background: var(--ifm-code-background);
              border-radius: 4px;
              border: 1px solid var(--ifm-color-emphasis-300);
            }

            .error-details summary {
              cursor: pointer;
              font-weight: 600;
              color: var(--ifm-color-primary);
              margin-bottom: 0.5rem;
            }

            .error-stack {
              margin-top: 1rem;
            }

            .error-stack h3 {
              font-size: 0.9rem;
              color: var(--ifm-color-danger);
              margin-bottom: 0.5rem;
            }

            .error-stack pre {
              font-size: 0.75rem;
              overflow-x: auto;
              padding: 0.5rem;
              background: var(--ifm-background-color);
              border-radius: 4px;
              color: var(--ifm-color-content-secondary);
            }

            .error-actions {
              display: flex;
              gap: 1rem;
              justify-content: center;
              margin-top: 1.5rem;
            }

            .button {
              padding: 0.75rem 1.5rem;
              border-radius: 6px;
              border: none;
              font-size: 1rem;
              cursor: pointer;
              transition: all 0.2s ease;
            }

            .button--primary {
              background: var(--ifm-color-primary);
              color: white;
            }

            .button--primary:hover {
              background: var(--ifm-color-primary-dark);
              transform: translateY(-2px);
            }

            .button--secondary {
              background: var(--ifm-color-emphasis-300);
              color: var(--ifm-color-content);
            }

            .button--secondary:hover {
              background: var(--ifm-color-emphasis-400);
              transform: translateY(-2px);
            }
          `}</style>
        </div>
      );
    }

    return this.props.children;
  }
}
