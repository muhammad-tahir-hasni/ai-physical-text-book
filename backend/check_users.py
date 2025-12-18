#!/usr/bin/env python3
"""
Check all users in database and verify their passwords.

Usage:
    python check_users.py
"""

import asyncio
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from sqlalchemy import select
from app.models.user import User
from app.services.auth_service import AuthService
from app.core.config import settings


async def check_all_users():
    """List all users and check their password hashes."""

    print("\n" + "=" * 70)
    print("🔍 DATABASE USER VERIFICATION")
    print("=" * 70)

    # Create engine and session
    engine = create_async_engine(settings.DATABASE_URL, echo=False)
    async_session = sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )

    async with async_session() as session:
        try:
            # Get all users
            result = await session.execute(select(User))
            users = result.scalars().all()

            if not users:
                print("\n❌ No users found in database!")
                print("\n💡 Create a test user:")
                print("   python create_test_user.py")
                return

            print(f"\n📊 Found {len(users)} user(s):\n")

            for i, user in enumerate(users, 1):
                print(f"{'─' * 70}")
                print(f"User #{i}")
                print(f"{'─' * 70}")
                print(f"   ID:           {user.id}")
                print(f"   Email:        {user.email}")
                print(f"   Role:         {user.role}")
                print(f"   Active:       {'✅ Yes' if user.is_active else '❌ No'}")
                print(f"   Created:      {user.created_at}")
                print(f"   Last Login:   {user.last_login or 'Never'}")
                print(f"   Password Hash: {user.password_hash[:60]}...")

                # Test password verification with common passwords
                test_passwords = ["Test123456", "Password123", "Admin123"]
                print(f"\n   🧪 Testing common passwords:")
                for test_pass in test_passwords:
                    is_valid = AuthService.verify_password(test_pass, user.password_hash)
                    status = "✅ MATCH" if is_valid else "❌ No match"
                    print(f"      '{test_pass}': {status}")
                print()

            print(f"{'─' * 70}")
            print("\n💡 To test login:")
            print("   1. Use one of the emails above")
            print("   2. Use the password that shows '✅ MATCH'")
            print("\n💡 To reset a user's password:")
            print("   python create_test_user.py")
            print("   (Enter the email and new password)")

        except Exception as e:
            print(f"\n❌ Error: {str(e)}")
            import traceback
            traceback.print_exc()
        finally:
            await engine.dispose()

    print("\n" + "=" * 70)


async def main():
    """Main function."""
    await check_all_users()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n⚠️  Cancelled by user")
