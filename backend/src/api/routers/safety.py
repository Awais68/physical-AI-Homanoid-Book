"""Safety monitoring API endpoints."""
from fastapi import APIRouter

router = APIRouter(prefix="/safety", tags=["safety"])


@router.get("/status")
async def get_safety_status():
    """Get current safety monitoring status."""
    return {
        "status": "active",
        "monitoring": True,
        "active_alerts": 0,
    }


@router.get("/health")
async def safety_health():
    """Check safety service health."""
    return {"status": "healthy", "service": "safety-monitoring"}
