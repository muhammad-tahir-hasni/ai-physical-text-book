#!/usr/bin/env python3
"""
Direct database user creation script.
Use this to create a test user if signup isn't working.

Usage:
    python create_test_user.py
"""

import asyncio
import sys
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from app.models.user import User
from app.services.auth_service import AuthService
from app.core.config import settings

async def create_test_user(email: str, password: str):
    """Create a test user directly in the database."""

    print(f"\n🔧 Creating test user...")
    print(f"   Email: {email}")
    print(f"   Password: {password}")
    print(f"   Database: {settings.DATABASE_URL.split('@')[1] if '@' in settings.DATABASE_URL else 'local'}")

    # Create engine and session
    engine = create_async_engine(settings.DATABASE_URL, echo=False)
    async_session = sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )

    async with async_session() as session:
        try:
            # Check if user already exists
            from sqlalchemy import select
            result = await session.execute(
                select(User).where(User.email == email)
            )
            existing_user = result.scalar_one_or_none()

            if existing_user:
                print(f"\n❌ User with email {email} already exists!")
                print(f"   User ID: {existing_user.id}")
                print(f"   Created: {existing_user.created_at}")

                # Ask to update password
                response = input(f"\n   Update password? (yes/no): ")
                if response.lower() in ['yes', 'y']:
                    hashed_password = AuthService.hash_password(password)
                    existing_user.password_hash = hashed_password
                    await session.commit()
                    print(f"\n✅ Password updated successfully!")
                    print(f"\n📝 You can now login with:")
                    print(f"   Email: {email}")
                    print(f"   Password: {password}")
                else:
                    print(f"\n   Keeping existing password.")
                return

            # Validate password
            if len(password) < 8:
                print(f"\n❌ Error: Password must be at least 8 characters")
                return

            if not any(c.isupper() for c in password):
                print(f"\n❌ Error: Password must contain at least 1 uppercase letter")
                return

            if not any(c.isdigit() for c in password):
                print(f"\n❌ Error: Password must contain at least 1 number")
                return

            # Hash password
            hashed_password = AuthService.hash_password(password)

            # Create user
            new_user = User(
                email=email,
                password_hash=hashed_password,
                role="user",
                is_active=True
            )

            session.add(new_user)
            await session.commit()
            await session.refresh(new_user)

            print(f"\n✅ Test user created successfully!")
            print(f"\n📝 Login credentials:")
            print(f"   Email: {email}")
            print(f"   Password: {password}")
            print(f"   User ID: {new_user.id}")
            print(f"\n🌐 Test at: http://localhost:3000")
            print(f"   1. Click 'Login' button")
            print(f"   2. Enter the email and password above")
            print(f"   3. Click 'Log In'")

        except Exception as e:
            print(f"\n❌ Error creating user: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            await engine.dispose()


async def main():
    """Main function."""
    print("=" * 60)
    print("🧪 Test User Creation Script")
    print("=" * 60)

    # Default test credentials
    default_email = "test@example.com"
    default_password = "Test123456"

    print(f"\nPress Enter to use defaults or type custom values:")
    print(f"Default Email: {default_email}")
    print(f"Default Password: {default_password}")

    email_input = input(f"\nEmail (or press Enter for default): ").strip()
    email = email_input if email_input else default_email

    password_input = input(f"Password (or press Enter for default): ").strip()
    password = password_input if password_input else default_password

    await create_test_user(email, password)

    print("\n" + "=" * 60)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  Cancelled by user")
        sys.exit(0)
