"""User management utility for development/testing."""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load backend .env
backend_dir = Path(__file__).parent
load_dotenv(backend_dir / ".env", override=True)

from app.core.database import get_db
from app.models.user import User
from sqlalchemy import select


async def list_users():
    """List all users in the database."""
    async for db in get_db():
        result = await db.execute(select(User))
        users = result.scalars().all()

        if not users:
            print("📭 No users found in database")
            return

        print(f"\n📋 Found {len(users)} user(s):\n")
        for user in users:
            print(f"  ID: {user.id}")
            print(f"  Email: {user.email}")
            print(f"  Role: {user.role}")
            print(f"  Active: {user.is_active}")
            print(f"  Created: {user.created_at}")
            print(f"  Last Login: {user.last_login}")
            print("-" * 50)
        break


async def delete_user_by_email(email: str):
    """Delete a user by email address."""
    async for db in get_db():
        result = await db.execute(
            select(User).where(User.email == email)
        )
        user = result.scalar_one_or_none()

        if not user:
            print(f"❌ User not found: {email}")
            return False

        print(f"\n⚠️  About to delete user:")
        print(f"  Email: {user.email}")
        print(f"  Created: {user.created_at}")

        confirm = input("\nAre you sure? (yes/no): ")
        if confirm.lower() != 'yes':
            print("❌ Deletion cancelled")
            return False

        await db.delete(user)
        await db.commit()
        print(f"✅ User deleted: {email}")
        return True
        break


async def delete_all_users():
    """Delete all users (dangerous!)."""
    async for db in get_db():
        result = await db.execute(select(User))
        users = result.scalars().all()

        if not users:
            print("📭 No users to delete")
            return

        print(f"\n⚠️  WARNING: About to delete {len(users)} user(s)!")
        for user in users:
            print(f"  - {user.email}")

        confirm = input(f"\nType 'DELETE ALL' to confirm: ")
        if confirm != 'DELETE ALL':
            print("❌ Deletion cancelled")
            return

        for user in users:
            await db.delete(user)

        await db.commit()
        print(f"✅ Deleted {len(users)} user(s)")
        break


async def main():
    """Main menu."""
    print("=" * 60)
    print("User Management Utility")
    print("=" * 60)
    print("\nOptions:")
    print("  1. List all users")
    print("  2. Delete user by email")
    print("  3. Delete all users (⚠️  dangerous!)")
    print("  4. Exit")

    choice = input("\nEnter choice (1-4): ").strip()

    if choice == "1":
        await list_users()
    elif choice == "2":
        email = input("Enter email to delete: ").strip()
        await delete_user_by_email(email)
    elif choice == "3":
        await delete_all_users()
    elif choice == "4":
        print("Goodbye!")
        sys.exit(0)
    else:
        print("❌ Invalid choice")


if __name__ == "__main__":
    asyncio.run(main())
