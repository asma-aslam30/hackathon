"""
Database migration scripts for user authentication and background collection.
This module provides functions to create the necessary database tables.
"""
from sqlalchemy import text
from ..config.database import sync_engine, Base
from ..models.user import User
from ..models.background import BackgroundInformation
from ..models.session import Session
from ..models.personalization import PersonalizationProfile

def create_tables():
    """
    Create all database tables based on the defined models.
    """
    print("Creating database tables...")

    # Create all tables
    Base.metadata.create_all(sync_engine)

    print("Database tables created successfully!")

def drop_tables():
    """
    Drop all database tables (use with caution!).
    """
    print("Dropping database tables...")

    # Drop all tables
    Base.metadata.drop_all(sync_engine)

    print("Database tables dropped successfully!")

def recreate_tables():
    """
    Drop and recreate all database tables.
    """
    print("Recreating database tables...")

    # Drop all tables
    Base.metadata.drop_all(sync_engine)

    # Create all tables
    Base.metadata.create_all(sync_engine)

    print("Database tables recreated successfully!")

def check_tables():
    """
    Check if the required tables exist in the database.
    """
    from sqlalchemy import inspect

    inspector = inspect(sync_engine)
    existing_tables = inspector.get_table_names()

    required_tables = [
        'users',
        'background_information',
        'sessions',
        'personalization_profiles'
    ]

    missing_tables = []
    for table in required_tables:
        if table not in existing_tables:
            missing_tables.append(table)

    if missing_tables:
        print(f"Missing tables: {missing_tables}")
        return False
    else:
        print("All required tables exist!")
        return True

if __name__ == "__main__":
    # If run directly, create the tables
    create_tables()