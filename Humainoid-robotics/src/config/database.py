"""Database configuration for Neon Postgres."""
try:
    from sqlalchemy import create_engine
    from sqlalchemy.ext.declarative import declarative_base
    from sqlalchemy.orm import sessionmaker
    from .settings import settings

    engine = create_engine(
        settings.DATABASE_URL,
        pool_pre_ping=True,
        echo=settings.DEBUG,
    )
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    Base = declarative_base()

    def get_db():
        """Get database session dependency."""
        db = SessionLocal()
        try:
            yield db
        finally:
            db.close()
except Exception as e:
    print(f"Database not available: {e}")
    from sqlalchemy.ext.declarative import declarative_base
    Base = declarative_base()
    engine = None
    SessionLocal = None

    def get_db():
        raise RuntimeError("Database not configured")
