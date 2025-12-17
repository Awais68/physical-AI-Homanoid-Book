from fastapi import FastAPI

app = FastAPI()

@app.get("/health")
def health():
    return {
        "status": "ok",
        "edge": "alive",
        "ros": "running"
    }
