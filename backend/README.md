# Physical AI & Humanoid Robotics Hackathon - Backend

FastAPI backend server providing authentication, user progress tracking, bookmarks, and RAG chatbot integration for the hackathon platform.

## Features

- **Authentication**: JWT-based signup/login with bcrypt password hashing (12 rounds)
- **User Management**: Role-based access control (participant/admin)
- **Progress Tracking**: Auto-save reading progress every 30s
- **Bookmarks**: Save and manage bookmarked sections
- **RAG Chatbot**: Qdrant vector database + Cohere embeddings + Claude/GPT-4 answer generation

## Technology Stack

- **Framework**: FastAPI 0.104.1
- **Database**: PostgreSQL 15+ with SQLAlchemy 2.0 async ORM
- **Caching**: Redis 7+ for rate limiting and session management
- **Vector DB**: Qdrant cloud for RAG embeddings
- **Authentication**: python-jose (JWT), passlib (bcrypt)

## Setup Instructions

### Prerequisites

- Python 3.11+
- PostgreSQL 15+
- Redis 7+
- Qdrant cloud account (or self-hosted instance)

### Installation

1. **Create virtual environment**:
   ```bash
   cd backend
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Environment configuration**:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

4. **Generate JWT secrets**:
   ```bash
   openssl rand -hex 32  # For JWT_SECRET_KEY
   openssl rand -hex 32  # For REFRESH_SECRET_KEY
   ```

5. **Database setup**:
   ```bash
   # Create database
   createdb hackathon_platform

   # Run migrations
   alembic upgrade head
   ```

### Running the Server

**Development mode** (with auto-reload):
```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

**Production mode**:
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

Server will be available at:
- API: http://localhost:8000
- Interactive docs: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql+asyncpg://user:pass@localhost:5432/hackathon_platform` |
| `REDIS_URL` | Redis connection string | `redis://localhost:6379/0` |
| `JWT_SECRET_KEY` | Secret key for access tokens | Generate with `openssl rand -hex 32` |
| `REFRESH_SECRET_KEY` | Secret key for refresh tokens | Generate with `openssl rand -hex 32` |
| `QDRANT_URL` | Qdrant instance URL | `https://xxx.cloud.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | From Qdrant dashboard |
| `COHERE_API_KEY` | Cohere API key for embeddings | From Cohere dashboard |
| `CLAUDE_API_KEY` | Claude API key for RAG answers | From Anthropic dashboard |
| `CORS_ORIGINS` | Allowed CORS origins (comma-separated) | `http://localhost:3000,http://localhost:8000` |

## API Endpoints

### Authentication

- `POST /auth/signup` - Create new user account
- `POST /auth/login` - Login and receive JWT tokens
- `POST /auth/refresh-token` - Refresh access token using refresh token
- `GET /auth/me` - Get current user info (requires JWT)

### Progress Tracking

- `POST /progress/save` - Save reading progress (requires JWT)
- `GET /progress/get` - Get user's current progress (requires JWT)

### Bookmarks

- `GET /bookmarks/user` - Get all user bookmarks (requires JWT)
- `POST /bookmarks` - Create bookmark (requires JWT)
- `DELETE /bookmarks/{bookmark_id}` - Delete bookmark (requires JWT)

### RAG Chatbot

- `POST /rag/query` - Query RAG chatbot with natural language (requires JWT)

## Database Migrations

Create new migration after model changes:
```bash
alembic revision --autogenerate -m "Description of changes"
```

Apply migrations:
```bash
alembic upgrade head
```

Rollback last migration:
```bash
alembic downgrade -1
```

## Testing

Run all tests:
```bash
pytest
```

Run with coverage:
```bash
pytest --cov=app --cov-report=html
```

Run specific test file:
```bash
pytest tests/api/test_auth.py
```

## Rate Limiting

- **Login endpoint**: 5 attempts per 15 minutes per IP
- **Signup endpoint**: 5 attempts per 15 minutes per IP
- Implemented using Redis sliding window

## Security Features

- **Password hashing**: bcrypt with 12 rounds minimum
- **JWT tokens**: HS256 algorithm, 1-hour access token, 7-day refresh token
- **httpOnly cookies**: Recommended for storing JWT in frontend
- **CORS**: Restricted to configured origins
- **Rate limiting**: Prevents brute force attacks
- **SQL injection protection**: Parameterized queries via SQLAlchemy ORM

## Project Structure

```
backend/
├── app/
│   ├── models/          # SQLAlchemy models
│   │   └── user.py
│   ├── schemas/         # Pydantic schemas
│   │   └── auth.py
│   ├── api/             # API route handlers
│   │   └── auth.py
│   ├── services/        # Business logic
│   │   └── auth_service.py
│   ├── middleware/      # Custom middleware
│   │   └── rate_limiter.py
│   ├── core/            # Core configuration
│   │   ├── config.py
│   │   └── database.py
│   └── main.py          # FastAPI app entry point
├── tests/               # Test suite
│   ├── api/
│   └── services/
├── db/
│   └── migrations/      # Alembic migrations
├── requirements.txt
├── alembic.ini
└── README.md
```

## Deployment

### Docker (Optional)

```bash
docker build -t hackathon-backend .
docker run -p 8000:8000 --env-file .env hackathon-backend
```

### Systemd Service (Linux)

Create `/etc/systemd/system/hackathon-backend.service`:
```ini
[Unit]
Description=Hackathon Backend API
After=network.target

[Service]
Type=simple
User=www-data
WorkingDirectory=/var/www/hackathon/backend
Environment="PATH=/var/www/hackathon/backend/venv/bin"
ExecStart=/var/www/hackathon/backend/venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable hackathon-backend
sudo systemctl start hackathon-backend
```

## Monitoring

View logs in development:
```bash
tail -f logs/app.log
```

Health check endpoint:
```bash
curl http://localhost:8000/health
```

## Troubleshooting

**Database connection errors**:
- Verify PostgreSQL is running: `sudo systemctl status postgresql`
- Check DATABASE_URL format: `postgresql+asyncpg://user:pass@host:port/db`

**Redis connection errors**:
- Verify Redis is running: `redis-cli ping` (should return `PONG`)
- Check REDIS_URL format: `redis://localhost:6379/0`

**JWT token errors**:
- Ensure JWT_SECRET_KEY and REFRESH_SECRET_KEY are set in .env
- Verify tokens are not expired (1 hour for access, 7 days for refresh)

## Contributing

1. Create feature branch from `main`
2. Follow PEP 8 style guidelines
3. Add tests for new features
4. Run tests before committing: `pytest`
5. Create pull request with detailed description

## License

MIT License - See LICENSE file for details
