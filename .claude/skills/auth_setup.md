# Auth Setup Skill

## Purpose

The Auth Setup skill provides a complete, production-ready authentication system boilerplate with JWT tokens, password hashing, user management, and secure session handling. This skill automates the creation of backend API endpoints, database models, middleware, and frontend authentication UI.

## Overview

This skill generates a full-stack authentication system including:

1. **Backend Components**: API endpoints, database models, JWT utilities, middleware
2. **Frontend Components**: Auth context, login/signup modals, protected routes
3. **Security Features**: Password hashing, token refresh, httpOnly cookies
4. **User Management**: Profile creation, background questionnaires, preferences

## When to Use

- **New Project Setup**: Bootstrap authentication for a greenfield project
- **Add Auth to Existing App**: Retrofit authentication into an existing application
- **Upgrade Auth System**: Modernize from session-based to JWT authentication
- **Multi-Tenant Setup**: Create authentication for multiple user types

## Prerequisites

```bash
# Backend dependencies
pip install fastapi sqlalchemy alembic passlib[bcrypt] python-jose python-multipart

# Frontend dependencies (React/TypeScript)
npm install react react-dom @types/react @types/react-dom

# Database
# Postgres with SQLAlchemy (asyncpg driver)
pip install asyncpg

# Environment variables
export JWT_SECRET_KEY="your-secret-key-here"
export JWT_ALGORITHM="HS256"
export ACCESS_TOKEN_EXPIRE_MINUTES=60
export REFRESH_TOKEN_EXPIRE_DAYS=7
```

## Skill Workflow

### Phase 1: Database Models

```python
"""
Create User and UserProfile models with SQLAlchemy.
"""

# backend/app/models/user.py
from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.orm import relationship
from datetime import datetime
from app.core.database import Base

class User(Base):
    """User account model."""
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String(255), unique=True, index=True, nullable=False)
    hashed_password = Column(String(255), nullable=False)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    # Relationships
    profile = relationship("UserProfile", back_populates="user", uselist=False)

# backend/app/models/user_profile.py
class UserProfile(Base):
    """User profile with optional background information."""
    __tablename__ = "user_profiles"

    user_id = Column(Integer, ForeignKey("users.id", ondelete="CASCADE"), primary_key=True)
    software_experience = Column(String(20), nullable=True)
    python_familiarity = Column(String(20), nullable=True)
    robotics_experience = Column(String(20), nullable=True)
    hardware_background = Column(String(20), nullable=True)
    learning_goals = Column(String(200), nullable=True)
    preferred_complexity = Column(String(20), default="intermediate", nullable=False)

    # Relationships
    user = relationship("User", back_populates="profile")
```

### Phase 2: Authentication Utilities

```python
"""
Password hashing and JWT token generation/verification.
"""

# backend/app/services/auth_service.py
from passlib.context import CryptContext
from jose import JWTError, jwt
from datetime import datetime, timedelta
from app.core.config import settings

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def hash_password(password: str) -> str:
    """Hash password using bcrypt."""
    return pwd_context.hash(password)

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify password against hash."""
    return pwd_context.verify(plain_password, hashed_password)

def create_access_token(data: dict, expires_delta: timedelta = None) -> str:
    """Create JWT access token."""
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES))
    to_encode.update({"exp": expire, "type": "access"})
    return jwt.encode(to_encode, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)

def create_refresh_token(data: dict) -> str:
    """Create JWT refresh token."""
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)
    to_encode.update({"exp": expire, "type": "refresh"})
    return jwt.encode(to_encode, settings.JWT_SECRET_KEY, algorithm=settings.JWT_ALGORITHM)

def verify_token(token: str) -> dict:
    """Verify and decode JWT token."""
    try:
        payload = jwt.decode(token, settings.JWT_SECRET_KEY, algorithms=[settings.JWT_ALGORITHM])
        return payload
    except JWTError:
        return None
```

### Phase 3: API Endpoints

```python
"""
Authentication API endpoints: signup, login, logout, refresh.
"""

# backend/app/api/auth.py
from fastapi import APIRouter, Depends, HTTPException, status, Response
from sqlalchemy.ext.asyncio import AsyncSession
from app.core.database import get_db
from app.services.auth_service import hash_password, verify_password, create_access_token, create_refresh_token
from app.models.user import User
from app.schemas.auth import SignupRequest, LoginRequest, TokenResponse

router = APIRouter()

@router.post("/auth/signup", response_model=TokenResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignupRequest, response: Response, db: AsyncSession = Depends(get_db)):
    """
    Create new user account.

    - Validates email uniqueness
    - Checks password strength (min 8 chars, 1 uppercase, 1 number)
    - Hashes password with bcrypt
    - Creates User and optional UserProfile
    - Returns JWT access + refresh tokens
    - Sets httpOnly cookie for additional security
    """
    # Check if user exists
    existing_user = await db.execute(select(User).where(User.email == request.email))
    if existing_user.scalar_one_or_none():
        raise HTTPException(status_code=400, detail="Email already registered")

    # Validate password strength
    if len(request.password) < 8:
        raise HTTPException(status_code=400, detail="Password must be at least 8 characters")

    # Create user
    user = User(
        email=request.email,
        hashed_password=hash_password(request.password)
    )
    db.add(user)
    await db.commit()
    await db.refresh(user)

    # Generate tokens
    access_token = create_access_token(data={"sub": user.email, "user_id": user.id})
    refresh_token = create_refresh_token(data={"sub": user.email, "user_id": user.id})

    # Set httpOnly cookie
    response.set_cookie(
        key="refresh_token",
        value=refresh_token,
        httponly=True,
        max_age=7*24*60*60,  # 7 days
        samesite="lax"
    )

    return TokenResponse(
        access_token=access_token,
        refresh_token=refresh_token,
        token_type="bearer",
        user={"id": user.id, "email": user.email}
    )

@router.post("/auth/login", response_model=TokenResponse)
async def login(request: LoginRequest, response: Response, db: AsyncSession = Depends(get_db)):
    """Authenticate user and return JWT tokens."""
    # Verify credentials
    user = await db.execute(select(User).where(User.email == request.email))
    user = user.scalar_one_or_none()

    if not user or not verify_password(request.password, user.hashed_password):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    # Generate tokens
    access_token = create_access_token(data={"sub": user.email, "user_id": user.id})
    refresh_token = create_refresh_token(data={"sub": user.email, "user_id": user.id})

    # Set httpOnly cookie
    response.set_cookie(key="refresh_token", value=refresh_token, httponly=True, max_age=7*24*60*60, samesite="lax")

    return TokenResponse(
        access_token=access_token,
        refresh_token=refresh_token,
        token_type="bearer",
        user={"id": user.id, "email": user.email}
    )

@router.post("/auth/logout")
async def logout(response: Response):
    """Clear authentication cookies."""
    response.delete_cookie(key="refresh_token")
    return {"message": "Logged out successfully"}

@router.post("/auth/refresh", response_model=TokenResponse)
async def refresh_token(refresh_token: str, db: AsyncSession = Depends(get_db)):
    """Refresh access token using refresh token."""
    payload = verify_token(refresh_token)
    if not payload or payload.get("type") != "refresh":
        raise HTTPException(status_code=401, detail="Invalid refresh token")

    # Generate new access token
    access_token = create_access_token(data={"sub": payload["sub"], "user_id": payload["user_id"]})

    return TokenResponse(access_token=access_token, token_type="bearer")
```

### Phase 4: Authentication Middleware

```python
"""
JWT verification middleware for protected routes.
"""

# backend/app/middleware/auth.py
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.ext.asyncio import AsyncSession
from app.core.database import get_db
from app.services.auth_service import verify_token
from app.models.user import User

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: AsyncSession = Depends(get_db)
) -> User:
    """
    Verify JWT token and return current user.

    Extracts token from Authorization header or httpOnly cookie.
    Verifies token signature and expiration.
    Fetches user from database.
    Raises 401 if invalid.
    """
    token = credentials.credentials

    payload = verify_token(token)
    if not payload:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    user_id = payload.get("user_id")
    user = await db.execute(select(User).where(User.id == user_id))
    user = user.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=401, detail="User not found")

    return user
```

### Phase 5: Frontend Authentication Context

```typescript
/**
 * AuthContext - Global authentication state management.
 * Provides login, signup, logout, and token refresh functions.
 */

// frontend/src/context/AuthContext.tsx
import React, { createContext, useState, useContext, useEffect } from 'react';

interface User {
  id: number;
  email: string;
}

interface AuthContextType {
  user: User | null;
  accessToken: string | null;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string) => Promise<void>;
  logout: () => void;
  refreshToken: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [accessToken, setAccessToken] = useState<string | null>(null);

  // Load tokens from localStorage on mount
  useEffect(() => {
    const storedToken = localStorage.getItem('access_token');
    const storedUser = localStorage.getItem('user');

    if (storedToken && storedUser) {
      setAccessToken(storedToken);
      setUser(JSON.parse(storedUser));
    }
  }, []);

  // Auto-refresh token before expiration (every 50 minutes)
  useEffect(() => {
    if (!accessToken) return;

    const interval = setInterval(() => {
      refreshToken();
    }, 50 * 60 * 1000); // 50 minutes

    return () => clearInterval(interval);
  }, [accessToken]);

  const login = async (email: string, password: string) => {
    const response = await fetch('/api/v1/auth/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
      credentials: 'include', // Include cookies
    });

    if (!response.ok) throw new Error('Login failed');

    const data = await response.json();
    setAccessToken(data.access_token);
    setUser(data.user);

    // Persist to localStorage
    localStorage.setItem('access_token', data.access_token);
    localStorage.setItem('user', JSON.stringify(data.user));
  };

  const signup = async (email: string, password: string) => {
    const response = await fetch('/api/v1/auth/signup', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
      credentials: 'include',
    });

    if (!response.ok) throw new Error('Signup failed');

    const data = await response.json();
    setAccessToken(data.access_token);
    setUser(data.user);

    localStorage.setItem('access_token', data.access_token);
    localStorage.setItem('user', JSON.stringify(data.user));
  };

  const logout = () => {
    setUser(null);
    setAccessToken(null);
    localStorage.removeItem('access_token');
    localStorage.removeItem('user');

    // Call backend logout to clear cookies
    fetch('/api/v1/auth/logout', { method: 'POST', credentials: 'include' });
  };

  const refreshToken = async () => {
    const response = await fetch('/api/v1/auth/refresh', {
      method: 'POST',
      credentials: 'include', // Send httpOnly cookie
    });

    if (!response.ok) {
      logout();
      return;
    }

    const data = await response.json();
    setAccessToken(data.access_token);
    localStorage.setItem('access_token', data.access_token);
  };

  return (
    <AuthContext.Provider value={{ user, accessToken, login, signup, logout, refreshToken }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};
```

### Phase 6: Protected Routes

```typescript
/**
 * ProtectedRoute - Wrapper for routes requiring authentication.
 */

// frontend/src/components/ProtectedRoute.tsx
import React from 'react';
import { Navigate } from 'react-router-dom';
import { useAuth } from '../context/AuthContext';

interface ProtectedRouteProps {
  children: React.ReactNode;
}

export default function ProtectedRoute({ children }: ProtectedRouteProps) {
  const { user } = useAuth();

  if (!user) {
    return <Navigate to="/login" replace />;
  }

  return <>{children}</>;
}
```

## Complete Automation Script

```bash
#!/bin/bash
# scripts/setup_auth.sh
# Generates complete authentication system boilerplate

set -e

echo "🔐 Setting up Authentication System..."

# Backend
echo "📦 Creating backend components..."
mkdir -p backend/app/models backend/app/api backend/app/services backend/app/middleware backend/app/schemas

# Generate files (content shown in phases above)
cat > backend/app/models/user.py << 'EOF'
# User model code here
EOF

cat > backend/app/services/auth_service.py << 'EOF'
# Auth service code here
EOF

# Frontend
echo "⚛️  Creating frontend components..."
mkdir -p frontend/src/context frontend/src/components

cat > frontend/src/context/AuthContext.tsx << 'EOF'
# AuthContext code here
EOF

# Database migration
echo "🗄️  Creating database migration..."
alembic revision --autogenerate -m "Add authentication tables"
alembic upgrade head

echo "✅ Authentication system setup complete!"
echo "   • Backend: API endpoints, models, middleware"
echo "   • Frontend: Auth context, protected routes"
echo "   • Security: JWT tokens, password hashing, httpOnly cookies"
```

## Usage Examples

### Generate Full Auth System

```bash
# One-command setup
./scripts/setup_auth.sh --project-name "physical-ai-textbook"
```

### Add Auth to Existing Backend

```bash
# Backend only
python scripts/generate_auth.py --backend-only --output backend/app
```

### Customize Token Expiration

```bash
# Set custom token lifetimes
export ACCESS_TOKEN_EXPIRE_MINUTES=30
export REFRESH_TOKEN_EXPIRE_DAYS=14
```

## Best Practices

1. **Password Security**:
   - Minimum 8 characters, 1 uppercase, 1 number
   - Bcrypt hashing with 12 rounds
   - Never log plaintext passwords

2. **Token Management**:
   - Access tokens: short-lived (60 minutes)
   - Refresh tokens: longer-lived (7 days)
   - Auto-refresh before expiration
   - Store refresh tokens in httpOnly cookies

3. **Session Handling**:
   - Implement token revocation for logout
   - Clear localStorage on logout
   - Handle expired tokens gracefully

4. **CORS Configuration**:
   - Allow credentials for cookie-based auth
   - Restrict origins to trusted domains
   - Set `SameSite=Lax` for CSRF protection

## Security Checklist

- [ ] Passwords hashed with bcrypt (12 rounds)
- [ ] JWT tokens signed with strong secret
- [ ] Access tokens expire after 60 minutes
- [ ] Refresh tokens stored in httpOnly cookies
- [ ] HTTPS enforced in production
- [ ] CORS configured with specific origins
- [ ] SQL injection prevented (parameterized queries)
- [ ] XSS mitigated (Content Security Policy)
- [ ] Rate limiting on auth endpoints

## Integration with Project

This skill powers:

- **Phase 5 (Authentication)**: Complete auth system implementation
- **User Profiles**: Background questionnaires and preferences
- **Protected Features**: Content personalization, saved chats
- **Analytics**: Track authenticated user behavior

## Troubleshooting

### Common Issues

1. **Token Verification Failed**:
   - Check JWT_SECRET_KEY matches between environments
   - Verify token hasn't expired
   - Ensure algorithm is HS256

2. **CORS Errors**:
   - Add frontend URL to CORS_ORIGINS
   - Set `allow_credentials=True`
   - Include credentials in fetch requests

3. **Cookie Not Set**:
   - Check `SameSite` and `Secure` flags
   - Verify domain matches
   - Ensure HTTPS in production

## Related Skills

- **RAG Setup**: Authentication protects RAG API endpoints
- **Content Generator**: Can generate auth tutorials
- **Diagram Describer**: Visualizes authentication flows
