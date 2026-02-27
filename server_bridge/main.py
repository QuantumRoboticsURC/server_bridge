# main.py
from fastapi import FastAPI, APIRouter
import uvicorn
import asyncio
from .app_globals import globals
from fastapi.middleware.cors import CORSMiddleware
from .ros2_bridge import start_ros2, stop_ros2
from .routes.ws_router import pos_router
from contextlib import asynccontextmanager
from .routes.ws_router import pos_router, register_ros2_callback
from .routes.autonomous import autonomous_router
from .routes.lab import lab_router, register_ros2_callback as register_lab_ros2_callback

@asynccontextmanager
async def lifespan(app: FastAPI):
    globals.loop = asyncio.get_running_loop()
    start_ros2()
    await asyncio.sleep(0.1)
    register_ros2_callback()
    register_lab_ros2_callback()
    yield
    stop_ros2()

app = FastAPI(lifespan=lifespan)
api = APIRouter()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/healthz")
async def healthz():
    return {"ok": True}

 #routes
api.include_router(pos_router, prefix="/ws/connection", tags=["connection"])
api.include_router(autonomous_router, prefix="/api/autonomous", tags=["autonomous"])
api.include_router(lab_router, prefix="/ws/connection", tags=["connection"])


#includes
app.include_router(api)


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main()