"""User management API endpoints."""
from fastapi import APIRouter

router = APIRouter(prefix="/users", tags=["users"])


@router.get("/profile")
async def get_user_profile():
    """Get current user profile."""
    return {
        "id": "demo-user",
        "name": "Demo User",
        "role": "educator",
    }


@router.get("/health")
async def users_health():
    """Check users service health."""
    return {"status": "healthy", "service": "user-management"}
